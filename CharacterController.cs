using System.Numerics;
using System.Collections.Generic;
using System.Threading.Tasks;

namespace SoftwareRenderer
{
    public class CharacterController
    {
        private readonly object CheckPlaneLock = new();
        private readonly object MoveWithSlideLock = new();

        // Core Properties
        public Vector3 Position { get; set; }
        public Vector3 Velocity { get; private set; }
        public bool IsGrounded { get; private set; }
        public bool IsCeiling { get; private set; }

        public bool IsNoClipEnabled { get; set; } = false;

        // Capsule Parameters
        public Vector3 Gravity { get; set; } = new(0, -14.0f, 0);
        public float Height { get; set; } = 0.5f;
        public float Radius { get; set; } = 0.15f;
        public float StepSize { get; set; } = 0.3f;
        public float MoveSpeed { get; set; } = 5.0f;
        public float JumpForce { get; set; } = 4f;
        public float GroundAcceleration { get; set; } = 3.5f;
        public float AirAcceleration { get; set; } = 0.35f;
        public float MaxAirSpeed { get; set; } = 6.0f;
        public float GroundFriction { get; set; } = 6.0f;
        public float AirControl { get; set; } = 0.2f;
        public Vector3 CamOffset { get; set; } = new(0.0f, 0.15f, 0.0f);

        // Environment
        private readonly List<Mesh>[] CollisionModels;
        private readonly Matrix4x4[] ModelMatrices;

        public CharacterController(Vector3 initialPosition, List<Mesh>[] collisionModels, Matrix4x4[] modelMatrices)
        {
            Position = initialPosition;
            Velocity = Vector3.Zero;
            CollisionModels = collisionModels;
            ModelMatrices = modelMatrices;
        }

        private float JumpCooldownTimer = 0f;
        private const float JumpCooldownDuration = 0.2f;

        public void Update(float DeltaTime, Vector3 MoveInput, bool JumpRequested)
        {
            if (IsNoClipEnabled)
            {
                // Direct movement ignoring physics and collisions
                Vector3 Dir = MoveInput;
                float InputMagnitude = Dir.Length();
                if (InputMagnitude > 1f) Dir /= InputMagnitude;
                Velocity = Dir * MoveSpeed;
                Position += Velocity * DeltaTime;
                return;
            }

            MoveInput.Y = 0;

            // Apply gravity
            Velocity += Gravity * DeltaTime;

            // Decrease jump cooldown timer
            if (JumpCooldownTimer > 0f)
            {
                JumpCooldownTimer -= DeltaTime;
            }

            // Handle jump input
            if (JumpRequested && IsGrounded && JumpCooldownTimer <= 0f)
            {
                Velocity = new Vector3(Velocity.X, JumpForce, Velocity.Z);
                IsGrounded = false;
                JumpCooldownTimer = JumpCooldownDuration;
            }

            // Ground check
            IsGrounded = CheckPlane(Position, CollisionModels, ModelMatrices, -1, out var GroundPoint,
                out var GroundNormal, Velocity, DeltaTime);

            Vector3 Movement = Velocity * DeltaTime;
            Vector3 MoveXZ = ProjectOnPlane(new Vector3(Movement.X, 0, Movement.Z), GroundNormal);

            // Ceiling check
            IsCeiling = CheckPlane(Position, CollisionModels, ModelMatrices, 1, out _, out _, Velocity, DeltaTime);

            // Ground response
            if (IsGrounded && GroundPoint != Vector3.NegativeInfinity)
            {
                Vector3 NewPosition = Position with { Y = GroundPoint.Y + Height * 0.5f };
                Position = MoveWithSlide(Position, NewPosition, Radius + 0.001f, 0, CollisionModels, ModelMatrices);

                if (Velocity.Y < 0)
                {
                    Velocity = new Vector3(Velocity.X, 0, Velocity.Z);
                }
            }

            // Ceiling response
            if (IsCeiling && Velocity.Y > 0)
            {
                Velocity = new Vector3(Velocity.X, 0, Velocity.Z);
                JumpCooldownTimer = 0;
            }

            // Horizontal slide movement
            Position = MoveWithSlide(Position, Position + MoveXZ, Radius + 0.001f, 0, CollisionModels, ModelMatrices);

            // Vertical movement
            Position += new Vector3(0, Velocity.Y, 0) * DeltaTime;

            // Handle input acceleration
            Vector3 WishDir = ProjectOnPlane(MoveInput, GroundNormal);
            float WishSpeed = WishDir.Length();
            if (WishSpeed > 1f) WishDir /= WishSpeed;
            WishSpeed *= MoveSpeed;

            if (IsGrounded)
            {
                ApplyFriction(DeltaTime);
                GroundAccelerate(WishDir, WishSpeed, DeltaTime);
            }
            else
            {
                AirAccelerate(WishDir, WishSpeed, DeltaTime);
                AirControlFunc(WishDir, DeltaTime);
                ClampAirSpeed();
            }
        }

        public static Vector3 ProjectOnPlane(Vector3 vector, Vector3 planeNormal)
        {
            float normalLengthSqr = planeNormal.X * planeNormal.X + planeNormal.Y * planeNormal.Y +
                                    planeNormal.Z * planeNormal.Z;
            if (normalLengthSqr < 1e-6f) return vector; // Return original vector if normal is invalid

            // Project: v - (v · n) * n (assuming planeNormal is normalized or close enough)
            float dot = vector.X * planeNormal.X + vector.Y * planeNormal.Y + vector.Z * planeNormal.Z;
            return new Vector3(
                vector.X - dot * planeNormal.X / normalLengthSqr,
                vector.Y - dot * planeNormal.Y / normalLengthSqr,
                vector.Z - dot * planeNormal.Z / normalLengthSqr
            );
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

        private bool CheckPlane(Vector3 pos, List<Mesh>[] collisionModels, Matrix4x4[] modelMatrices, float direction,
            out Vector3 point, out Vector3 normal, Vector3 velocity, float deltaTime)
        {
            point = Vector3.NegativeInfinity;
            normal = Vector3.UnitY;
            if (collisionModels == null || modelMatrices == null || collisionModels.Length != modelMatrices.Length)
                return false;

            Vector3[] offsets =
            {
                new Vector3(0, 0, 0),
                new Vector3(-1, 0, 0),
                new Vector3(1, 0, 0),
                new Vector3(0, 0, -1),
                new Vector3(0, 0, 1),
                new Vector3(-1, 0, -1),
                new Vector3(-1, 0, 1),
                new Vector3(1, 0, -1),
                new Vector3(1, 0, 1)
            };

            Vector3 bestHitPoint = Vector3.NegativeInfinity;
            Vector3 bestNormal = Vector3.UnitY;
            float bestDistance = float.MaxValue;
            bool anyHit = false;

            object hitLock = new();

            Vector3 frameStart = pos;
            Vector3 frameEnd = pos + new Vector3(0, velocity.Y, 0) * deltaTime;
            float maxDistance = MathF.Abs(frameEnd.Y - frameStart.Y) + (Height);

            Parallel.ForEach(offsets, offset =>
            {
                Vector3 safeOffset =
                    offset == Vector3.Zero ? Vector3.Zero : Vector3.Normalize(offset) * (Radius - 0.01f);
                Vector3 heightOffset = Vector3.UnitY * direction * (Height / 2f);

                Vector3 rayStart = frameStart + safeOffset - heightOffset;
                Vector3 rayEnd = frameEnd + safeOffset + heightOffset;
                Vector3 rayDir = rayEnd - rayStart;

                if (rayDir.LengthSquared() < 0.0001f)
                    return;

                Vector3 normRayDir = Vector3.Normalize(rayDir);

                for (int i = 0; i < collisionModels.Length; i++)
                {
                    var model = collisionModels[i];
                    var matrix = modelMatrices[i];

                    foreach (var mesh in model)
                    {
                        if (Physics.Raycast(rayStart, normRayDir, mesh.Vertices.ToArray(), mesh.Indices.ToArray(),
                                matrix, out float hitDistance, out var hitPos, out var hitNormal))
                        {
                            if (hitDistance <= maxDistance)
                            {
                                lock (hitLock)
                                {
                                    if (hitDistance < bestDistance)
                                    {
                                        bestDistance = hitDistance;
                                        bestHitPoint = hitPos;
                                        bestNormal = hitNormal;
                                        anyHit = true;
                                    }
                                }
                            }
                        }
                    }
                }
            });

            point = bestHitPoint;
            normal = bestNormal;
            return anyHit;
        }

        private Vector3 MoveWithSlide(Vector3 currentPos, Vector3 desiredPos, float radius, int depth,
            List<Mesh>[] sceneModels = null, Matrix4x4[] modelMatrices = null)
        {
            const int MaxSlideAttempts = 3;
            const float SkinWidth = 0.01f;

            if (depth >= MaxSlideAttempts || sceneModels == null || modelMatrices == null ||
                sceneModels.Length != modelMatrices.Length)
                return currentPos;

            Vector3 moveVector = desiredPos - currentPos;
            float moveDistance = moveVector.Length();

            Vector3 direction = Vector3.Normalize(moveVector);
            float nearestHitDistance = moveDistance;
            Vector3 hitNormal = Vector3.Zero;
            bool collisionDetected = false;

            float capsuleHalfHeight = Height * 0.5f;
            int verticalSteps = Math.Max(1, (int)(Height / (radius * 2)));
            int horizontalRays = Math.Max(3, (int)(4 * MathF.PI * radius / 0.1f));

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
                        float heightOffset = float.Lerp(bottomLimit, capsuleHalfHeight,
                            vStep / (float)Math.Max(1, verticalSteps));

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
                                lock (MoveWithSlideLock)
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

            Vector3 slideDirection = Vector3.Cross(hitNormal, Vector3.Cross(remainingMove, hitNormal));
            if (slideDirection == Vector3.Zero)
                return safeStopPos;

            slideDirection = Vector3.Normalize(slideDirection) * remainingMove.Length();

            Vector3 adjustedSlideTarget = safeStopPos + slideDirection;
            return MoveWithSlide(safeStopPos, adjustedSlideTarget, radius, depth + 1, sceneModels, modelMatrices);
        }
    }
}