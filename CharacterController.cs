using System.Numerics;

namespace SoftwareRenderer
{
    public class CharacterController
    {
        public Vector3 Position { get; private set; }
        public Vector3 Velocity { get; private set; }
        public bool IsGrounded { get; private set; }

        private float Height = 0.5f;
        private float Radius = 0.15f;
        private float Mass = 80f;
        private float GroundCheckDistance = 0.1f;
        private float GroundFriction = 8f;
        private float AirControl = 0.5f;
        private float JumpForce = 3f;
        private float MoveSpeed = 2f;
        private float RunMultiplier = 2f;
        private Vector3 Gravity = new Vector3(0, -9.81f, 0);

        // Accept collision meshes *and* their model matrices in parallel arrays
        private List<Mesh>[] CollisionModels;
        private Matrix4x4[] ModelMatrices;

        // Constructor now accepts model matrices too
        public CharacterController(Vector3 initialPosition, List<Mesh>[] collisionModels, Matrix4x4[] modelMatrices)
        {
            Position = initialPosition;
            Velocity = Vector3.Zero;
            CollisionModels = collisionModels;
            ModelMatrices = modelMatrices;
        }

        public void Update(float deltaTime, Vector3 moveInput, bool jumpRequested, bool isRunning)
        {
            Velocity += new Vector3(0, Gravity.Y * deltaTime, 0);
            float speed = isRunning ? MoveSpeed * RunMultiplier : MoveSpeed;
            Vector3 desiredVelocity = moveInput * speed;

            if (IsGrounded)
            {
                Velocity = new Vector3(
                    MathHelper.Lerp(Velocity.X, desiredVelocity.X, GroundFriction * deltaTime),
                    Velocity.Y,
                    MathHelper.Lerp(Velocity.Z, desiredVelocity.Z, GroundFriction * deltaTime)
                );
            }
            else
            {
                Velocity = new Vector3(
                    MathHelper.Lerp(Velocity.X, desiredVelocity.X, GroundFriction * deltaTime),
                    Velocity.Y + desiredVelocity.Y * AirControl * deltaTime,
                    MathHelper.Lerp(Velocity.Z, desiredVelocity.Z, GroundFriction * deltaTime)
                );
            }

            if (jumpRequested && IsGrounded)
            {
                Velocity = Velocity with { Y = JumpForce };
                IsGrounded = false;
            }

            Vector3 movement = Velocity * deltaTime;
            Vector3 horizontalMovement = new Vector3(movement.X, 0, movement.Z);
            Vector3 verticalMovement = new Vector3(0, movement.Y, 0);

            Position = MoveWithSlide(Position, Position + horizontalMovement, Radius, 0, CollisionModels, ModelMatrices);
            Position = MoveWithSlide(Position, Position + verticalMovement, Radius, 0, CollisionModels, ModelMatrices);
            IsGrounded = CheckGrounded(CollisionModels, ModelMatrices);
        }

        private bool CheckGrounded(List<Mesh>[] collisionModels, Matrix4x4[] modelMatrices)
        {
            Vector3 rayStart = Position;
            Vector3 rayEnd = rayStart + Vector3.UnitY * -(Height * 0.5f + GroundCheckDistance);

            bool hit = false;

            if (collisionModels != null && modelMatrices != null && collisionModels.Length == modelMatrices.Length)
            {
                for (int i = 0; i < collisionModels.Length; i++)
                {
                    var model = collisionModels[i];
                    var modelMatrix = modelMatrices[i];

                    Parallel.ForEach(model.ToArray(), mesh =>
                    {
                        if (Physics.Raycast(rayStart, Vector3.Normalize(rayEnd - rayStart),
                                mesh.Vertices.ToArray(), mesh.Indices.ToArray(), modelMatrix,
                                out float hitDistance, out var hitPoint, out _))
                        {
                            if (hitDistance <= (Height * 0.5f + GroundCheckDistance))
                            {
                                Position = Position with
                                {
                                    Y = hitPoint.Y + Height * 0.5f + GroundCheckDistance
                                };
                                Velocity = Velocity with { Y = 0 };
                                hit = true;
                            }
                        }
                    });
                    if (hit) break;
                }
            }

            return hit;
        }

        private Vector3 MoveWithSlide(Vector3 currentPos, Vector3 desiredPos, float radius, int depth,
            List<Mesh>[] sceneModels = null, Matrix4x4[] modelMatrices = null)
        {
            const int MaxSlideAttempts = 3;
            const float MinMoveDistance = 0.001f;
            const float SkinWidth = 0.01f;

            if (depth >= MaxSlideAttempts || sceneModels == null || sceneModels.Length == 0
                || modelMatrices == null || modelMatrices.Length != sceneModels.Length)
                return currentPos;

            Vector3 moveVector = desiredPos - currentPos;
            float moveDistance = moveVector.Length();

            if (moveDistance < MinMoveDistance)
                return currentPos;

            Vector3 direction = moveVector / moveDistance;
            float nearestHitDistance = moveDistance;
            Vector3 hitNormal = Vector3.Zero;
            bool collisionDetected = false;

            float capsuleHalfHeight = Height * 0.5f - radius;
            int verticalSteps = Math.Max(1, (int)(Height / (radius * 2)));
            int horizontalRays = Math.Max(3, (int)(2 * Math.PI * radius / 0.1f));

            object hitLock = new object();

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
                        float heightOffset = MathHelper.Lerp(-capsuleHalfHeight, capsuleHalfHeight,
                            vStep / (float)verticalSteps);

                        for (int hStep = 0; hStep < horizontalRays; hStep++)
                        {
                            float angle = 2 * MathF.PI * hStep / horizontalRays;
                            Vector3 horizontalOffset = new Vector3(
                                radius * MathF.Cos(angle),
                                0,
                                radius * MathF.Sin(angle)
                            );

                            Vector3 rayOrigin = currentPos + new Vector3(0, heightOffset, 0) + horizontalOffset;

                            if (Physics.Raycast(rayOrigin, direction, vertices, indices, modelMatrix,
                                    out float hitDistance, out _, out Vector3 normal) &&
                                hitDistance < nearestHitDistance)
                            {
                                lock (hitLock)
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