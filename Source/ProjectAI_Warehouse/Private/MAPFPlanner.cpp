#include "MAPFPlanner.h"
#include "Math/UnrealMathUtility.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetSystemLibrary.h"
#include "Engine/World.h"

// ============================================================================
// INITIALIZATION
// ============================================================================

void UMAPFPlanner::InitializeGridCache()
{
    GridCache.Empty();

    UWorld* World = GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF: GetWorld() returned nullptr in InitializeGridCache"));
        return;
    }

    int32 TotalCells = 0;
    int32 ObstacleCells = 0;

    for (int32 X = GridMinX; X <= GridMaxX; X++)
    {
        for (int32 Y = GridMinY; Y <= GridMaxY; Y++)
        {
            FIntVector GridPos(X, Y, 0);
            FVector WorldPos = GridToWorld(GridPos);

            FGridCell Cell;
            Cell.bIsWalkable = true;
            Cell.bIsObstacle = CheckStaticObstacle(WorldPos);

            if (Cell.bIsObstacle)
            {
                ObstacleCells++;
            }

            GridCache.Add(GridPos, Cell);
            TotalCells++;
        }
    }

    UE_LOG(LogTemp, Log, TEXT("MAPF: Grid initialized. Total: %d cells, Obstacles: %d (%.1f%%)"),
        TotalCells, ObstacleCells, (TotalCells > 0) ? (100.0f * ObstacleCells / TotalCells) : 0.0f);
}

// ============================================================================
// OBSTACLE CHECKING
// ============================================================================

bool UMAPFPlanner::CheckStaticObstacle(FVector Position) const
{
    UWorld* World = GetWorld();
    if (!World) return true;

    if (!bUseLineTraceForObstacles)
    {
        return false;
    }

    FVector Start = Position;
    Start.Z += 200.0f;
    FVector End = Position;
    End.Z = -100.0f;

    FHitResult Hit;
    FCollisionQueryParams QueryParams;
    QueryParams.bTraceComplex = false;
    QueryParams.bReturnPhysicalMaterial = false;

    bool bHit = World->LineTraceSingleByChannel(
        Hit, Start, End, ECC_WorldStatic, QueryParams
    );

    if (!bHit)
    {
        return true;
    }

    float HitZ = Hit.Location.Z;
    float TargetZ = Position.Z;

    if (FMath::Abs(HitZ - TargetZ) > 50.0f)
    {
        return true;
    }

    return false;
}

bool UMAPFPlanner::IsCellPassable(FVector WorldPosition) const
{
    FIntVector GridPos = WorldToGrid(WorldPosition);
    if (!IsWithinGridBounds(GridPos))
    {
        return false;
    }

    if (GridCache.Num() > 0)
    {
        if (const FGridCell* Cell = GridCache.Find(GridPos))
        {
            return Cell->bIsWalkable && !Cell->bIsObstacle;
        }
    }

    return !CheckStaticObstacle(WorldPosition);
}

bool UMAPFPlanner::IsWithinGridBounds(const FIntVector& GridPos) const
{
    return (GridPos.X >= GridMinX && GridPos.X <= GridMaxX &&
        GridPos.Y >= GridMinY && GridPos.Y <= GridMaxY);
}

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

uint64 UMAPFPlanner::PackKey(int32 X, int32 Y, int32 T) const
{
    // Pack as: [X(20 bits)][Y(20 bits)][T(24 bits)]
    uint64 PackedX = (uint64)(X + 524288) & 0xFFFFF; // 20 bits, offset by 2^19
    uint64 PackedY = (uint64)(Y + 524288) & 0xFFFFF;
    uint64 PackedT = (uint64)(T + 8388608) & 0xFFFFFF; // 24 bits, offset by 2^23

    return (PackedX << 44) | (PackedY << 24) | PackedT;
}

void UMAPFPlanner::UnpackKey(uint64 Key, int32& OutX, int32& OutY, int32& OutT) const
{
    OutX = (int32)((Key >> 44) & 0xFFFFF) - 524288;
    OutY = (int32)((Key >> 24) & 0xFFFFF) - 524288;
    OutT = (int32)(Key & 0xFFFFFF) - 8388608;
}

FIntVector UMAPFPlanner::WorldToGrid(FVector Pos) const
{
    return FIntVector(
        FMath::FloorToInt(Pos.X / (CellSizeMeters * 100.0f)),
        FMath::FloorToInt(Pos.Y / (CellSizeMeters * 100.0f)),
        0
    );
}

FVector UMAPFPlanner::GridToWorld(FIntVector GridPos) const
{
    return FVector(
        (GridPos.X + 0.5f) * CellSizeMeters * 100.0f,
        (GridPos.Y + 0.5f) * CellSizeMeters * 100.0f,
        0.0f
    );
}

float UMAPFPlanner::Heuristic(const FIntVector& Current, const FIntVector& Goal) const
{
    // Octile distance for better diagonal estimation
    int32 dx = FMath::Abs(Goal.X - Current.X);
    int32 dy = FMath::Abs(Goal.Y - Current.Y);
    return (dx + dy) + (FMath::Sqrt(2.0f) - 2.0f) * FMath::Min(dx, dy);
}

TArray<FIntVector> UMAPFPlanner::GetFootprintCells(int32 X, int32 Y, int32 T) const
{
    TArray<FIntVector> Cells;
    int32 Offset = AgentClearance;
    Cells.Reserve((Offset * 2 + 1) * (Offset * 2 + 1));

    for (int32 dx = -Offset; dx <= Offset; ++dx)
    {
        for (int32 dy = -Offset; dy <= Offset; ++dy)
        {
            Cells.Add(FIntVector(X + dx, Y + dy, T));
        }
    }
    return Cells;
}

// ============================================================================
// CONFLICT CHECKING WITH PRIORITY
// ============================================================================

float UMAPFPlanner::CalculateConflictPenalty(const FIntVector& Loc, int32 AgentID, float Priority) const
{
    uint64 Key = PackKey(Loc.X, Loc.Y, Loc.Z);

    if (const FReservationEntry* Entry = ReservationTable.Find(Key))
    {
        if (Entry->AgentID == AgentID)
        {
            return 0.0f;  // Own reservation
        }

        // Higher priority agents get lower penalties
        if (Priority < Entry->Priority)
        {
            return ConflictCostFactor * 0.1f;  // Can override lower priority
        }
        else
        {
            return ConflictCostFactor;  // Full penalty
        }
    }

    return 0.0f;  // No reservation
}

bool UMAPFPlanner::CheckVertexConflict(const FIntVector& Loc, int32 AgentID, float Priority) const
{
    TArray<FIntVector> Footprint = GetFootprintCells(Loc.X, Loc.Y, Loc.Z);

    for (const FIntVector& Cell : Footprint)
    {
        uint64 Key = PackKey(Cell.X, Cell.Y, Cell.Z);
        if (const FReservationEntry* Entry = ReservationTable.Find(Key))
        {
            if (Entry->AgentID != AgentID)
            {
                // Check if we can override based on priority
                if (Priority >= Entry->Priority && bEnableCooperativePlanning)
                {
                    continue;  // Can override
                }
                return true;  // Vertex conflict
            }
        }
    }
    return false;
}

bool UMAPFPlanner::CheckEdgeConflict(const FIntVector& From, const FIntVector& To, int32 T, int32 AgentID, float Priority) const
{
    TArray<FIntVector> FootFrom = GetFootprintCells(From.X, From.Y, T);
    TArray<FIntVector> FootTo = GetFootprintCells(To.X, To.Y, T + 1);

    for (const FIntVector& C1 : FootFrom)
    {
        for (const FIntVector& C2 : FootTo)
        {
            uint64 KeySwap1 = PackKey(C2.X, C2.Y, T);
            uint64 KeySwap2 = PackKey(C1.X, C1.Y, T + 1);

            const FReservationEntry* Entry1 = ReservationTable.Find(KeySwap1);
            const FReservationEntry* Entry2 = ReservationTable.Find(KeySwap2);

            if (Entry1 && Entry2 && Entry1->AgentID == Entry2->AgentID && Entry1->AgentID != AgentID)
            {
                if (Priority >= Entry1->Priority && bEnableCooperativePlanning)
                {
                    continue;  // Can override
                }
                return true;  // Edge conflict (swap)
            }
        }
    }
    return false;
}

bool UMAPFPlanner::TryStealReservation(int32 NewAgentID, int32 OldAgentID, const FIntVector& Location, float NewPriority)
{
    uint64 Key = PackKey(Location.X, Location.Y, Location.Z);

    if (FReservationEntry* Entry = ReservationTable.Find(Key))
    {
        if (Entry->AgentID == OldAgentID && NewPriority < Entry->Priority)
        {
            Entry->AgentID = NewAgentID;
            Entry->Priority = NewPriority;
            Entry->Timestamp = GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0f;

            UE_LOG(LogTemp, Log, TEXT("MAPF: Agent %d stole reservation from Agent %d at (%d,%d,%d)"),
                NewAgentID, OldAgentID, Location.X, Location.Y, Location.Z);
            return true;
        }
    }
    return false;
}

// ============================================================================
// COOPERATIVE A* WITH TIME WINDOWS
// ============================================================================

TArray<FVector> UMAPFPlanner::PlanPath(int32 AgentID, FVector StartWorld, FVector GoalWorld, float StartTime, float Priority)
{
    UWorld* World = GetWorld();
    if (!World)
    {
        return TArray<FVector>();
    }

    float CurrentWorldTime = World->GetTimeSeconds();
    CleanupStaleReservations(CurrentWorldTime);

    FIntVector StartGrid = WorldToGrid(StartWorld);
    StartGrid.Z = FMath::CeilToInt(StartTime / TimeStepSec);

    FIntVector GoalGrid = WorldToGrid(GoalWorld);

    // Check if goal is reachable
    if (!IsCellPassable(GoalWorld))
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF: Goal position (%.1f, %.1f) is not passable for Agent %d!"),
            GoalWorld.X, GoalWorld.Y, AgentID);
        return TArray<FVector>();
    }

    // Data structures for Cooperative A*
    TArray<FSpaceTimeNode> Nodes;
    Nodes.Reserve(MaxSearchNodes);

    TSet<int32> OpenSet;
    TSet<int32> ClosedSet;
    TMap<uint64, int32> GridToNodeIdx;

    // Start node
    FSpaceTimeNode StartNode;
    StartNode.Loc = StartGrid;
    StartNode.G = 0;
    StartNode.H = Heuristic(StartGrid, GoalGrid);
    StartNode.F = StartNode.H;
    StartNode.ParentIndex = INDEX_NONE;
    StartNode.bIsWait = false;
    StartNode.WaitCount = 0;

    int32 StartIdx = Nodes.Add(StartNode);
    OpenSet.Add(StartIdx);
    GridToNodeIdx.Add(PackKey(StartGrid.X, StartGrid.Y, StartGrid.Z), StartIdx);

    // Movement actions with waiting penalized
    TArray<FIntVector> Deltas = {
        FIntVector(0, 0, 0),   // Wait (penalized)
        FIntVector(1, 0, 0),   // Right
        FIntVector(-1, 0, 0),  // Left
        FIntVector(0, 1, 0),   // Forward
        FIntVector(0, -1, 0)   // Backward
    };

    int32 GoalIndex = INDEX_NONE;
    int32 NodesExplored = 0;

    // Main search loop
    while (OpenSet.Num() > 0 && Nodes.Num() < MaxSearchNodes)
    {
        // Find node with minimum F
        int32 CurrentIdx = INDEX_NONE;
        float MinF = TNumericLimits<float>::Max();

        for (int32 Idx : OpenSet)
        {
            if (Nodes[Idx].F < MinF)
            {
                MinF = Nodes[Idx].F;
                CurrentIdx = Idx;
            }
        }

        if (CurrentIdx == INDEX_NONE) break;

        OpenSet.Remove(CurrentIdx);
        ClosedSet.Add(CurrentIdx);
        NodesExplored++;

        const FSpaceTimeNode& Current = Nodes[CurrentIdx];
        int32 CurTime = Current.Loc.Z;

        // Check goal reached
        if (Current.Loc.X == GoalGrid.X && Current.Loc.Y == GoalGrid.Y)
        {
            GoalIndex = CurrentIdx;
            break;
        }

        // Generate successors
        for (const FIntVector& Delta : Deltas)
        {
            FIntVector NextLoc = FIntVector(
                Current.Loc.X + Delta.X,
                Current.Loc.Y + Delta.Y,
                CurTime + 1
            );

            // Time horizon check
            if (NextLoc.Z > StartGrid.Z + MaxTimeHorizon)
            {
                continue;
            }

            // Grid bounds check
            if (!IsWithinGridBounds(NextLoc))
            {
                continue;
            }

            // Static obstacle check
            FVector NextWorldPos = GridToWorld(NextLoc);
            if (!IsCellPassable(NextWorldPos))
            {
                continue;
            }

            bool bIsWait = (Delta.X == 0 && Delta.Y == 0);

            // Prevent infinite waiting
            if (bIsWait && Current.WaitCount >= MaxConsecutiveWaits)
            {
                continue;
            }

            // Calculate movement cost
            float MoveCost = bIsWait ? WaitCostFactor : 1.0f;

            // Check vertex conflicts with priority
            float VertexPenalty = CalculateConflictPenalty(NextLoc, AgentID, Priority);
            if (VertexPenalty >= ConflictCostFactor)
            {
                // Can't override this reservation
                continue;
            }
            MoveCost += VertexPenalty;

            // Check edge conflicts (only for moves)
            if (!bIsWait)
            {
                if (CheckEdgeConflict(Current.Loc, NextLoc, CurTime, AgentID, Priority))
                {
                    continue;
                }
            }

            // Calculate total cost
            float TentativeG = Current.G + MoveCost;
            float H = Heuristic(NextLoc, GoalGrid);
            float F = TentativeG + H;

            uint64 NextKey = PackKey(NextLoc.X, NextLoc.Y, NextLoc.Z);
            bool bBetter = false;
            int32 ExistingIdx = INDEX_NONE;

            if (int32* FoundIdx = GridToNodeIdx.Find(NextKey))
            {
                ExistingIdx = *FoundIdx;
                if (TentativeG < Nodes[ExistingIdx].G)
                {
                    bBetter = true;
                    if (ClosedSet.Contains(ExistingIdx))
                    {
                        ClosedSet.Remove(ExistingIdx);
                        OpenSet.Add(ExistingIdx);
                    }
                }
            }

            if (bBetter || ExistingIdx == INDEX_NONE)
            {
                FSpaceTimeNode NewNode;
                NewNode.Loc = NextLoc;
                NewNode.G = TentativeG;
                NewNode.H = H;
                NewNode.F = F;
                NewNode.ParentIndex = CurrentIdx;
                NewNode.bIsWait = bIsWait;
                NewNode.WaitCount = bIsWait ? Current.WaitCount + 1 : 0;

                if (ExistingIdx != INDEX_NONE)
                {
                    Nodes[ExistingIdx] = NewNode;
                }
                else
                {
                    ExistingIdx = Nodes.Add(NewNode);
                    OpenSet.Add(ExistingIdx);
                    GridToNodeIdx.Add(NextKey, ExistingIdx);
                }
            }
        }
    }

    // Reconstruct path
    TArray<FVector> WorldPath;
    if (GoalIndex != INDEX_NONE)
    {
        TArray<FSpaceTimeNode> ReversePath;
        int32 Curr = GoalIndex;
        while (Curr != INDEX_NONE)
        {
            ReversePath.Add(Nodes[Curr]);
            Curr = Nodes[Curr].ParentIndex;
        }

        // Reverse to get forward path
        for (int32 i = ReversePath.Num() - 1; i >= 0; --i)
        {
            WorldPath.Add(GridToWorld(ReversePath[i].Loc));
        }

        UE_LOG(LogTemp, Log, TEXT("MAPF: Path found for Agent %d - %d points, %d nodes explored, %d waits"),
            AgentID, WorldPath.Num(), NodesExplored,
            [&]() { int32 w = 0; for (const auto& n : ReversePath) if (n.bIsWait) w++; return w; }());
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF: Path NOT FOUND for Agent %d after exploring %d nodes"),
            AgentID, NodesExplored);
    }

    return WorldPath;
}

// ============================================================================
// RESERVATION MANAGEMENT
// ============================================================================

bool UMAPFPlanner::ReservePath(int32 AgentID, const TArray<FIntVector>& Path, float Priority)
{
    if (Path.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF: Cannot reserve empty path for Agent %d"), AgentID);
        return false;
    }

    UWorld* World = GetWorld();
    float CurrentTime = World ? World->GetTimeSeconds() : 0.0f;

    // Try to reserve all cells
    for (const FIntVector& Loc : Path)
    {
        TArray<FIntVector> Foot = GetFootprintCells(Loc.X, Loc.Y, Loc.Z);

        for (const FIntVector& Cell : Foot)
        {
            uint64 Key = PackKey(Cell.X, Cell.Y, Cell.Z);

            if (FReservationEntry* Existing = ReservationTable.Find(Key))
            {
                if (Existing->AgentID != AgentID)
                {
                    // Check if we can override
                    if (Priority < Existing->Priority)
                    {
                        // Steal reservation
                        Existing->AgentID = AgentID;
                        Existing->Priority = Priority;
                        Existing->Timestamp = CurrentTime;
                    }
                    else
                    {
                        UE_LOG(LogTemp, Warning, TEXT("MAPF: Reservation conflict at (%d,%d,%d) - Agent %d vs %d"),
                            Cell.X, Cell.Y, Cell.Z, AgentID, Existing->AgentID);
                        return false;
                    }
                }
            }
            else
            {
                ReservationTable.Add(Key, FReservationEntry(AgentID, Priority, CurrentTime));
            }
        }
    }

    UE_LOG(LogTemp, Log, TEXT("MAPF: Path reserved for Agent %d (%d points, priority %.2f)"),
        AgentID, Path.Num(), Priority);
    return true;
}

void UMAPFPlanner::ReleasePath(int32 AgentID, const TArray<FIntVector>& Path)
{
    int32 ReleasedCount = 0;

    for (const FIntVector& Loc : Path)
    {
        TArray<FIntVector> Foot = GetFootprintCells(Loc.X, Loc.Y, Loc.Z);

        for (const FIntVector& Cell : Foot)
        {
            uint64 Key = PackKey(Cell.X, Cell.Y, Cell.Z);

            if (FReservationEntry* Entry = ReservationTable.Find(Key))
            {
                if (Entry->AgentID == AgentID)
                {
                    ReservationTable.Remove(Key);
                    ReleasedCount++;
                }
            }
        }
    }

    if (ReleasedCount > 0)
    {
        UE_LOG(LogTemp, Verbose, TEXT("MAPF: Released %d cells for Agent %d"), ReleasedCount, AgentID);
    }
}

void UMAPFPlanner::CleanupStaleReservations(float CurrentTime)
{
    const float StaleThreshold = 30.0f; // 30 seconds
    TArray<uint64> StaleKeys;

    for (const auto& Pair : ReservationTable)
    {
        if (CurrentTime - Pair.Value.Timestamp > StaleThreshold)
        {
            StaleKeys.Add(Pair.Key);
        }
    }

    for (uint64 Key : StaleKeys)
    {
        ReservationTable.Remove(Key);
    }

    if (StaleKeys.Num() > 0)
    {
        UE_LOG(LogTemp, Log, TEXT("MAPF: Cleaned up %d stale reservations"), StaleKeys.Num());
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

UMAPFPlanner* UMAPFPlanner::GetPlanner(const UObject* WorldContextObject)
{
    if (!WorldContextObject)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF: GetPlanner - WorldContextObject is null"));
        return nullptr;
    }

    UWorld* World = WorldContextObject->GetWorld();
    if (!World)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF: GetPlanner - World is null"));
        return nullptr;
    }

    UGameInstance* GameInstance = World->GetGameInstance();
    if (!GameInstance)
    {
        UE_LOG(LogTemp, Error, TEXT("MAPF: GetPlanner - GameInstance is null"));
        return nullptr;
    }

    return GameInstance->GetSubsystem<UMAPFPlanner>();
}

void UMAPFPlanner::DebugPrintReservations() const
{
    UE_LOG(LogTemp, Log, TEXT("=== MAPF Reservation Table (%d entries) ==="), ReservationTable.Num());

    for (const auto& Pair : ReservationTable)
    {
        int32 X, Y, T;
        UnpackKey(Pair.Key, X, Y, T);
        UE_LOG(LogTemp, Log, TEXT("  (%d,%d,%d) -> Agent %d, Priority %.2f"),
            X, Y, T, Pair.Value.AgentID, Pair.Value.Priority);
    }
}

void UMAPFPlanner::DrawDebugGrid(float Duration, bool bDrawCells)
{
    UWorld* World = GetWorld();
    if (!World) return;

    FVector MinWorld = GridToWorld(FIntVector(GridMinX, GridMinY, 0));
    FVector MaxWorld = GridToWorld(FIntVector(GridMaxX, GridMaxY, 0));
    MinWorld.Z = 5.0f;
    MaxWorld.Z = 5.0f;

    FVector Center = (MinWorld + MaxWorld) * 0.5f;
    FVector Extent = (MaxWorld - MinWorld) * 0.5f;
    DrawDebugBox(World, Center, FVector(Extent.X, Extent.Y, 2.0f), FColor::Red, false, Duration, 0, 3.0f);

    if (bDrawCells)
    {
        int32 Step = FMath::Max(1, (GridMaxX - GridMinX) / 20);
        for (int32 X = GridMinX; X <= GridMaxX; X += Step)
        {
            for (int32 Y = GridMinY; Y <= GridMaxY; Y += Step)
            {
                FVector CellCenter = GridToWorld(FIntVector(X, Y, 0));
                CellCenter.Z = 2.0f;

                FColor Color = FColor::Green;
                uint64 Key = PackKey(X, Y, 0);
                if (ReservationTable.Contains(Key))
                {
                    Color = FColor::Orange;
                }

                DrawDebugSphere(World, CellCenter, 3.0f, 4, Color, false, Duration);
            }
        }
    }

    UE_LOG(LogTemp, Log, TEXT("MAPF Debug: Grid bounds drawn. X:[%d..%d] Y:[%d..%d]"),
        GridMinX, GridMaxX, GridMinY, GridMaxY);
}