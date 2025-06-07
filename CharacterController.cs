using System.Numerics;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace SoftwareRenderer
{
    public class CharacterController
    {
        private readonly object _checkPlaneLock = new();
        private readonly object _moveWithSlideLock = new();

        // Core Properties
        public Vector3 Position { get; set; }
        public Vector3 Velocity { get; private set; }
        public bool IsGrounded { get; private set; }
        public bool IsCeiling { get; private set; }

        // Capsule Parameters
        public Vector3 Gravity { get; set; } = new(0, -14.0f, 0);
        public float Height { get; set; } = 0.5f;
        public float Radius { get; set; } = 0.15f;
        public float StepSize { get; set; } = 0.2f;
        public float GroundCheckDistance { get; set; } = 0.05f;
        public float MoveSpeed { get; set; } = 5.0f;
        public float JumpForce { get; set; } = 4f;
        public float GroundAcceleration { get; set; } = 3.5f;
        public float AirAcceleration { get; set; } = 0.35f;
        public float MaxAirSpeed { get; set; } = 6.0f;
        public float GroundFriction { get; set; } = 6.0f;
        public float AirControl { get; set; } = 0.2f;

        // Environment
        private readonly List<Mesh>[] _collisionModels;
        private readonly Matrix4x4[] _modelMatrices;

        public CharacterController(Vector3 initialPosition, List<Mesh>[] collisionModels, Matrix4x4[] modelMatrices)
        {
            Position = initialPosition;
            Velocity = Vector3.Zero;
            _collisionModels = collisionModels;
            _modelMatrices = modelMatrices;
        }

        public void Update(float deltaTime, Vector3 moveInput, bool jumpRequested, bool isRunning)
        {
            Velocity += Gravity * deltaTime;

            Vector3 wishDir = moveInput;
            float wishSpeed = wishDir.Length();
            if (wishSpeed > 1f) wishDir /= wishSpeed;
            wishSpeed *= MoveSpeed;

            if (IsGrounded)
            {
                ApplyFriction(deltaTime);
                GroundAccelerate(wishDir, wishSpeed, deltaTime);
            }
            else
            {
                AirAccelerate(wishDir, wishSpeed, deltaTime);
                AirControlFunc(wishDir, deltaTime);
                ClampAirSpeed();
            }

            if (jumpRequested && IsGrounded)
            {
                Velocity = new Vector3(Velocity.X, JumpForce, Velocity.Z);
                IsGrounded = false;
            }

            Vector3 movement = Velocity * deltaTime;
            Vector3 moveXZ = new(movement.X, 0, movement.Z);
            Vector3 moveY = new(0, movement.Y, 0);

            Position = MoveWithSlide(Position, Position + moveXZ, Radius + 0.001f, 0, _collisionModels, _modelMatrices);
            Position += moveY;

            IsGrounded = CheckPlane(_collisionModels, _modelMatrices, -1, out var groundPoint);

            if (IsGrounded && groundPoint != Vector3.NegativeInfinity)
            {
                Position = Position with { Y = groundPoint.Y + Height * 0.5f + GroundCheckDistance };
                if (Velocity.Y < 0)
                    Velocity = new Vector3(Velocity.X, 0, Velocity.Z);
            }

            IsCeiling = CheckPlane(_collisionModels, _modelMatrices, 1, out _);
            if (IsCeiling && Velocity.Y > 0)
            {
                Velocity = new Vector3(Velocity.X, 0, Velocity.Z);
            }
        }

        private void ApplyFriction(float deltaTime)
        {
            Vector3 horizontalVel = new(Velocity.X, 0, Velocity.Z);
            float speed = horizontalVel.Length();
            if (speed < 0.1f)
            {
                Velocity = new Vector3(0, Velocity.Y, 0);
                return;
            }

            float drop = speed * GroundFriction * deltaTime;
            float newSpeed = MathF.Max(speed - drop, 0);
            float scale = newSpeed / speed;
            Velocity = new Vector3(Velocity.X * scale, Velocity.Y, Velocity.Z * scale);
        }

        private void GroundAccelerate(Vector3 wishDir, float wishSpeed, float deltaTime)
        {
            Vector3 horizontalVel = new(Velocity.X, 0, Velocity.Z);
            float currentSpeed = Vector3.Dot(horizontalVel, wishDir);
            float addSpeed = wishSpeed - currentSpeed;
            if (addSpeed <= 0) return;

            float accelSpeed = MathF.Min(GroundAcceleration * wishSpeed * deltaTime, addSpeed);
            Velocity += new Vector3(wishDir.X * accelSpeed, 0, wishDir.Z * accelSpeed);
        }

        private void AirAccelerate(Vector3 wishDir, float wishSpeed, float deltaTime)
        {
            Vector3 horizontalVel = new(Velocity.X, 0, Velocity.Z);
            float currentSpeed = Vector3.Dot(horizontalVel, wishDir);
            float addSpeed = wishSpeed - currentSpeed;
            if (addSpeed <= 0) return;

            float accelSpeed = MathF.Min(AirAcceleration * wishSpeed * deltaTime, addSpeed);

            Vector3 projectedVel = horizontalVel + wishDir * accelSpeed;
            if (projectedVel.Length() > MaxAirSpeed)
            {
                projectedVel = Vector3.Normalize(projectedVel) * MaxAirSpeed;
                Velocity = new Vector3(projectedVel.X, Velocity.Y, projectedVel.Z);
            }
            else
            {
                Velocity += new Vector3(wishDir.X * accelSpeed, 0, wishDir.Z * accelSpeed);
            }
        }

        private void ClampAirSpeed()
        {
            Vector3 horizontalVel = new(Velocity.X, 0, Velocity.Z);
            float speed = horizontalVel.Length();
            if (speed > MaxAirSpeed)
            {
                horizontalVel = Vector3.Normalize(horizontalVel) * MaxAirSpeed;
                Velocity = new Vector3(horizontalVel.X, Velocity.Y, horizontalVel.Z);
            }
        }

        private void AirControlFunc(Vector3 wishDir, float deltaTime)
        {
            if (wishDir.LengthSquared() < 0.001f) return;

            Vector3 horizontalVel = new(Velocity.X, 0, Velocity.Z);
            float speed = horizontalVel.Length();
            if (speed < 0.1f) return;

            float k = AirControl * deltaTime;
            Velocity += new Vector3(wishDir.X * k, 0, wishDir.Z * k);
        }

        private bool CheckPlane(List<Mesh>[] collisionModels, Matrix4x4[] modelMatrices, float direction, out Vector3 point)
        {
            Vector3 rayStart = Position;
            Vector3 rayEnd = rayStart + Vector3.UnitY * direction * (Height * 0.501f + GroundCheckDistance);

            bool hit = false;
            Vector3 hitPoint = Vector3.NegativeInfinity;

            if (collisionModels != null && modelMatrices != null && collisionModels.Length == modelMatrices.Length)
            {
                for (int i = 0; i < collisionModels.Length; i++)
                {
                    var model = collisionModels[i];
                    var modelMatrix = modelMatrices[i];

                    Parallel.ForEach(model.ToArray(), mesh =>
                    {
                        lock (_checkPlaneLock)
                        {
                            Vector3 rayDir = rayEnd - rayStart;
                            if (rayDir.LengthSquared() > 0 &&
                                Physics.Raycast(rayStart, Vector3.Normalize(rayDir),
                                    mesh.Vertices.ToArray(), mesh.Indices.ToArray(), modelMatrix,
                                    out float hitDistance, out var currentHitPoint, out _))
                            {
                                if (hitDistance <= (Height * 0.5f + GroundCheckDistance))
                                {
                                    lock (_checkPlaneLock)
                                    {
                                        if (!hit || hitDistance < Vector3.Distance(rayStart, hitPoint))
                                        {
                                            hitPoint = currentHitPoint;
                                            hit = true;
                                        }
                                    }
                                }
                            }
                        }
                    });

                    if (hit) break;
                }
            }

            point = hitPoint;
            return hit;
        }



        private Vector3 MoveWithSlide(Vector3 currentPos, Vector3 desiredPos, float radius, int depth,
            List<Mesh>[] sceneModels = null, Matrix4x4[] modelMatrices = null)
        {
            const int MaxSlideAttempts = 3;
            const float MinMoveDistance = 0.01f;
            const float SkinWidth = 0.01f;

            if (depth >= MaxSlideAttempts || sceneModels == null || modelMatrices == null || sceneModels.Length != modelMatrices.Length)
                return currentPos;

            Vector3 moveVector = desiredPos - currentPos;
            float moveDistance = moveVector.Length();
            if (moveDistance < MinMoveDistance) return currentPos;

            Vector3 direction = Vector3.Normalize(moveVector);
            float nearestHitDistance = moveDistance;
            Vector3 hitNormal = Vector3.Zero;
            bool collisionDetected = false;

            float capsuleHalfHeight = Height * 0.5f;
            int verticalSteps = Math.Max(1, (int)(Height / (radius * 2)));
            int horizontalRays = Math.Max(3, (int)(2 * MathF.PI * radius / 0.1f));

            for (int i = 0; i < sceneModels.Length; i++)
            {
                var model = sceneModels[i];
                var modelMatrix = modelMatrices[i];

                Parallel.ForEach(model.ToArray(), mesh =>
                {
                    var vertices = mesh.Vertices.ToArray();
                    var indices = mesh.Indices.ToArray();

                    for (int vStep = 0; vStep <= verticalSteps; vStep++)
                    {
                        float bottomLimit = -capsuleHalfHeight + StepSize;
                        float heightOffset = float.Lerp(bottomLimit, capsuleHalfHeight, vStep / (float)Math.Max(1, verticalSteps));

                        for (int hStep = 0; hStep < horizontalRays; hStep++)
                        {
                            float angle = 2 * MathF.PI * hStep / horizontalRays;
                            Vector3 horizontalOffset = new(
                                radius * MathF.Cos(angle),
                                0,
                                radius * MathF.Sin(angle)
                            );

                            Vector3 rayOrigin = currentPos + new Vector3(0, heightOffset, 0) + horizontalOffset;

                            if (Physics.Raycast(rayOrigin, direction, vertices, indices, modelMatrix,
                                out float hitDistance, out _, out Vector3 normal))
                            {
                                lock (_moveWithSlideLock)
                                {
                                    if (hitDistance < nearestHitDistance)
                                    {
                                        nearestHitDistance = hitDistance;
                                        hitNormal = Vector3.Normalize(normal);
                                        collisionDetected = true;
                                    }
                                }
                            }
                        }
                    }
                });
            }

            if (!collisionDetected)
                return desiredPos;

            Vector3 safeStopPos = currentPos + direction * (nearestHitDistance - SkinWidth);
            Vector3 remainingMove = desiredPos - safeStopPos;

            float alignment = Vector3.Dot(direction, hitNormal);
            if (MathF.Abs(alignment) > 0.9f)
                return safeStopPos;

            if (remainingMove.LengthSquared() < MinMoveDistance * MinMoveDistance)
                return safeStopPos;

            Vector3 slideDirection = Vector3.Cross(hitNormal, Vector3.Cross(remainingMove, hitNormal));
            if (slideDirection == Vector3.Zero)
                return safeStopPos;

            slideDirection = Vector3.Normalize(slideDirection) * remainingMove.Length();
            if (slideDirection.LengthSquared() < MinMoveDistance * MinMoveDistance)
                return safeStopPos;

            Vector3 adjustedSlideTarget = safeStopPos + slideDirection;
            return MoveWithSlide(safeStopPos, adjustedSlideTarget, radius, depth + 1, sceneModels, modelMatrices);
        }
    }
}
