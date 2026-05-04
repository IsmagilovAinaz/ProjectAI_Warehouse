#include "MAPFPlanner.h"
#include "Math/UnrealMathUtility.h"
#include "Algo/Reverse.h"
#include "DrawDebugHelpers.h"
#include "Kismet/KismetSystemLibrary.h"

// ============================================================================
// ИНИЦИАЛИЗАЦИЯ СЕТКИ
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

    // Проходим по всей сетке склада
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
// ПРОВЕРКА ПРЕПЯТСТВИЙ
// ============================================================================

bool UMAPFPlanner::CheckStaticObstacle(FVector Position) const
{
    UWorld* World = GetWorld();
    if (!World) return true; // Считаем непроходимым, если нет мира

    if (!bUseLineTraceForObstacles)
    {
        // Быстрая проверка: просто проверяем границы
        return false;
    }

    // LineTrace вниз для обнаружения пола и стен
    FVector Start = Position;
    Start.Z += 200.0f; // Начинаем высоко

    FVector End = Position;
    End.Z = -100.0f;   // Заканчиваем низко

    FHitResult Hit;
    FCollisionQueryParams QueryParams;
    QueryParams.bTraceComplex = false;
    QueryParams.bReturnPhysicalMaterial = false;

    // Игнорируем самих роботов (Pawn)
    QueryParams.AddIgnoredActor(nullptr);

    // Трассируем через все статические объекты
    bool bHit = World->LineTraceSingleByChannel(
        Hit,
        Start,
        End,
        ECC_WorldStatic,
        QueryParams
    );

    if (!bHit)
    {
        // Если ничего не нашли - это пустота (не проходимо)
        return true;
    }

    // Проверяем, что попали в пол, а не в стену
    // Если hit произошел близко к нашей точке Z - это пол (проходимо)
    // Если hit произошел высоко - это стена (не проходимо)
    float HitZ = Hit.Location.Z;
    float TargetZ = Position.Z;

    // Если разница по Z больше 50 см - это вероятно стена
    if (FMath::Abs(HitZ - TargetZ) > 50.0f)
    {
        return true; // Стена
    }

    return false; // Пол - проходимо
}

bool UMAPFPlanner::IsCellPassable(FVector WorldPosition) const
{
    // 1. Проверяем границы сетки
    FIntVector GridPos = WorldToGrid(WorldPosition);
    if (!IsWithinGridBounds(GridPos))
    {
        return false;
    }

    // 2. Проверяем кэш (если инициализирован)
    if (GridCache.Num() > 0)
    {
        if (const FGridCell* Cell = GridCache.Find(GridPos))
        {
            return Cell->bIsWalkable && !Cell->bIsObstacle;
        }
    }

    // 3. Если кэш пуст - проверяем напрямую
    return !CheckStaticObstacle(WorldPosition);
}

bool UMAPFPlanner::IsWithinGridBounds(const FIntVector& GridPos) const
{
    return (GridPos.X >= GridMinX && GridPos.X <= GridMaxX &&
        GridPos.Y >= GridMinY && GridPos.Y <= GridMaxY);
}

// ============================================================================
// ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
// ============================================================================

uint64 UMAPFPlanner::PackKey(int32 X, int32 Y, int32 T) const
{
    // Упаковываем координаты в uint64 для быстрого поиска
    // Смещения гарантируют положительные значения
    return ((uint64)(X + 10000) << 32) | ((uint64)(Y + 10000) << 16) | (uint64)(T + 1000);
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
    // Манхэттенское расстояние (адаптировано для 4-связного графа)
    return static_cast<float>(FMath::Abs(Goal.X - Current.X) + FMath::Abs(Goal.Y - Current.Y));
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
// ПРОВЕРКА КОНФЛИКТОВ
// ============================================================================

bool UMAPFPlanner::CheckVertexConflict(const FIntVector& Loc, int32 AgentID) const
{
    TArray<FIntVector> Footprint = GetFootprintCells(Loc.X, Loc.Y, Loc.Z);

    for (const FIntVector& Cell : Footprint)
    {
        uint64 Key = PackKey(Cell.X, Cell.Y, Cell.Z);
        if (ReservationTable.Contains(Key) && ReservationTable[Key] != AgentID)
        {
            return true; // Вершинный конфликт обнаружен
        }
    }
    return false;
}

bool UMAPFPlanner::CheckEdgeConflict(const FIntVector& From, const FIntVector& To, int32 T, int32 AgentID) const
{
    // Проверяем размен позициями (edge conflict)
    TArray<FIntVector> FootFrom = GetFootprintCells(From.X, From.Y, T);
    TArray<FIntVector> FootTo = GetFootprintCells(To.X, To.Y, T + 1);

    for (const FIntVector& C1 : FootFrom)
    {
        for (const FIntVector& C2 : FootTo)
        {
            uint64 KeySwap1 = PackKey(C2.X, C2.Y, T);
            uint64 KeySwap2 = PackKey(C1.X, C1.Y, T + 1);

            if (ReservationTable.Contains(KeySwap1) && ReservationTable.Contains(KeySwap2))
            {
                int32 OtherID = ReservationTable[KeySwap1];
                if (OtherID != AgentID && OtherID == ReservationTable[KeySwap2])
                {
                    return true; // Рёберный конфликт (swap)
                }
            }
        }
    }
    return false;
}

// ============================================================================
// ОСНОВНОЙ АЛГОРИТМ A* В ПРОСТРАНСТВЕ-ВРЕМЕНИ
// ============================================================================

TArray<FVector> UMAPFPlanner::PlanPath(int32 AgentID, FVector StartWorld, FVector GoalWorld, float StartTime)
{
    // Преобразуем в сетку
    FIntVector StartGrid = WorldToGrid(StartWorld);
    StartGrid.Z = FMath::CeilToInt(StartTime / TimeStepSec);

    FIntVector GoalGrid = WorldToGrid(GoalWorld);

    // Проверяем проходимость цели
    if (!IsCellPassable(GoalWorld))
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF: Goal position (%.1f, %.1f) is not passable!"),
            GoalWorld.X, GoalWorld.Y);
        return TArray<FVector>();
    }

    // Структуры данных для A*
    TArray<FSpaceTimeNode> Nodes;
    Nodes.Reserve(MaxSearchNodes);

    TSet<int32> OpenSet;
    TSet<int32> ClosedSet;
    TMap<uint64, int32> GridToNodeIdx;

    // Стартовый узел
    FSpaceTimeNode StartNode;
    StartNode.Loc = StartGrid;
    StartNode.G = 0;
    StartNode.H = Heuristic(StartGrid, GoalGrid);
    StartNode.F = StartNode.H;
    StartNode.ParentIndex = INDEX_NONE;
    StartNode.bIsWait = false;

    int32 StartIdx = Nodes.Add(StartNode);
    OpenSet.Add(StartIdx);
    GridToNodeIdx.Add(PackKey(StartGrid.X, StartGrid.Y, StartGrid.Z), StartIdx);

    // Возможные действия: ждать, двигаться в 4 направлениях
    TArray<FIntVector> Deltas = {
        FIntVector(0, 0, 0),   // Wait
        FIntVector(1, 0, 0),   // Right
        FIntVector(-1, 0, 0),  // Left
        FIntVector(0, 1, 0),   // Up
        FIntVector(0, -1, 0)   // Down
    };

    int32 GoalIndex = INDEX_NONE;
    int32 BlockedCellsCount = 0;
    int32 ConflictCount = 0;

    // Основной цикл A*
    while (OpenSet.Num() > 0 && Nodes.Num() < MaxSearchNodes)
    {
        // 1. Выбираем узел с минимальным F
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

        const FSpaceTimeNode& Current = Nodes[CurrentIdx];
        int32 CurTime = Current.Loc.Z;

        // 2. Проверка достижения цели
        if (Current.Loc.X == GoalGrid.X && Current.Loc.Y == GoalGrid.Y)
        {
            GoalIndex = CurrentIdx;
            break;
        }

        // 3. Генерация потомков
        for (const FIntVector& Delta : Deltas)
        {
            FIntVector NextLoc = FIntVector(
                Current.Loc.X + Delta.X,
                Current.Loc.Y + Delta.Y,
                CurTime + 1
            );

            // Проверка временного горизонта
            if (NextLoc.Z > StartGrid.Z + MaxTimeHorizon)
            {
                continue;
            }

            // Проверка границ сетки
            if (!IsWithinGridBounds(NextLoc))
            {
                continue;
            }

            // 🔥 ПРОВЕРКА СТАТИЧЕСКИХ ПРЕПЯТСТВИЙ
            FVector NextWorldPos = GridToWorld(NextLoc);
            if (!IsCellPassable(NextWorldPos))
            {
                BlockedCellsCount++;
                continue;
            }

            bool bIsWait = (Delta.X == 0 && Delta.Y == 0);

            // Проверка вершинных конфликтов
            if (CheckVertexConflict(NextLoc, AgentID))
            {
                ConflictCount++;
                continue;
            }

            // Проверка рёберных конфликтов (только для перемещений)
            if (!bIsWait && CheckEdgeConflict(Current.Loc, NextLoc, CurTime, AgentID))
            {
                ConflictCount++;
                continue;
            }

            // Вычисление стоимости
            float TentativeG = Current.G + 1.0f;
            float H = Heuristic(NextLoc, GoalGrid);
            float F = TentativeG + H;

            uint64 NextKey = PackKey(NextLoc.X, NextLoc.Y, NextLoc.Z);
            bool bBetter = false;
            int32 ExistingIdx = INDEX_NONE;

            // Проверка, существует ли уже этот узел
            if (GridToNodeIdx.Contains(NextKey))
            {
                ExistingIdx = GridToNodeIdx[NextKey];

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

            // Добавляем или обновляем узел
            if (bBetter || ExistingIdx == INDEX_NONE)
            {
                FSpaceTimeNode NewNode;
                NewNode.Loc = NextLoc;
                NewNode.G = TentativeG;
                NewNode.H = H;
                NewNode.F = F;
                NewNode.ParentIndex = CurrentIdx;
                NewNode.bIsWait = bIsWait;

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

    // Логирование статистики
    if (BlockedCellsCount > 0 || ConflictCount > 0)
    {
        UE_LOG(LogTemp, Log, TEXT("MAPF: Agent %d - Blocked: %d, Conflicts: %d, Nodes: %d"),
            AgentID, BlockedCellsCount, ConflictCount, Nodes.Num());
    }

    // Восстановление пути
    TArray<FVector> WorldPath;
    if (GoalIndex != INDEX_NONE)
    {
        int32 Curr = GoalIndex;
        while (Curr != INDEX_NONE)
        {
            WorldPath.Insert(GridToWorld(Nodes[Curr].Loc), 0);
            Curr = Nodes[Curr].ParentIndex;
        }
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF: Path NOT FOUND for Agent %d. Goal unreachable or fully blocked."), AgentID);
    }

    return WorldPath;
}

// ============================================================================
// РЕЗЕРВИРОВАНИЕ И ОСВОБОЖДЕНИЕ ПУТИ
// ============================================================================

bool UMAPFPlanner::ReservePath(int32 AgentID, const TArray<FIntVector>& Path)
{
    if (Path.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("MAPF: Cannot reserve empty path for Agent %d"), AgentID);
        return false;
    }

    // 1. Проверка всех ячеек на конфликты
    for (const FIntVector& Loc : Path)
    {
        TArray<FIntVector> Foot = GetFootprintCells(Loc.X, Loc.Y, Loc.Z);

        for (const FIntVector& Cell : Foot)
        {
            uint64 Key = PackKey(Cell.X, Cell.Y, Cell.Z);

            if (ReservationTable.Contains(Key) && ReservationTable[Key] != AgentID)
            {
                UE_LOG(LogTemp, Warning, TEXT("MAPF: Reservation conflict at (%d,%d,t=%d) for Agent %d"),
                    Cell.X, Cell.Y, Cell.Z, AgentID);
                return false;
            }
        }
    }

    // 2. Атомарное резервирование
    for (const FIntVector& Loc : Path)
    {
        TArray<FIntVector> Foot = GetFootprintCells(Loc.X, Loc.Y, Loc.Z);

        for (const FIntVector& Cell : Foot)
        {
            uint64 Key = PackKey(Cell.X, Cell.Y, Cell.Z);
            ReservationTable.Add(Key, AgentID);
        }
    }

    UE_LOG(LogTemp, Log, TEXT("MAPF: Path reserved for Agent %d (%d points, clearance=%d)"),
        AgentID, Path.Num(), AgentClearance);

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

            if (ReservationTable.Contains(Key) && ReservationTable[Key] == AgentID)
            {
                ReservationTable.Remove(Key);
                ReleasedCount++;
            }
        }
    }

    UE_LOG(LogTemp, Verbose, TEXT("MAPF: Released %d cells for Agent %d"), ReleasedCount, AgentID);
}

// ============================================================================
// ПОЛУЧЕНИЕ ПЛАНИРОВЩИКА
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

void UMAPFPlanner::DrawDebugGrid(float Duration, bool bDrawCells)
{
    UWorld* World = GetWorld();
    if (!World) return;

    // Мировые координаты углов сетки
    FVector MinWorld = GridToWorld(FIntVector(GridMinX, GridMinY, 0));
    FVector MaxWorld = GridToWorld(FIntVector(GridMaxX, GridMaxY, 0));

    // Поднимаем над полом для наглядности
    MinWorld.Z = 5.0f;
    MaxWorld.Z = 5.0f;

    // 1. Рисуем границы сетки (красный прямоугольник)
    FVector Center = (MinWorld + MaxWorld) * 0.5f;
    FVector Extent = (MaxWorld - MinWorld) * 0.5f;
    DrawDebugBox(World, Center, FVector(Extent.X, Extent.Y, 2.0f), FColor::Red, false, Duration, 0, 3.0f);

    // 2. Опционально: рисуем центры ячеек (зелёные точки)
    if (bDrawCells)
    {
        // Рисуем каждую 5-ю ячейку, чтобы не просадить FPS
        int32 Step = FMath::Max(1, (GridMaxX - GridMinX) / 15);
        for (int32 X = GridMinX; X <= GridMaxX; X += Step)
        {
            for (int32 Y = GridMinY; Y <= GridMaxY; Y += Step)
            {
                FVector CellCenter = GridToWorld(FIntVector(X, Y, 0));
                CellCenter.Z = 2.0f;
                DrawDebugSphere(World, CellCenter, 3.0f, 4, FColor::Green, false, Duration);
            }
        }
    }

    UE_LOG(LogTemp, Log, TEXT("MAPF Debug: Grid bounds drawn. X:[%d..%d] Y:[%d..%d]"),
        GridMinX, GridMaxX, GridMinY, GridMaxY);
}