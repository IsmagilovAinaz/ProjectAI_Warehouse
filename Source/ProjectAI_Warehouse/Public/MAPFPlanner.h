#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "Math/UnrealMathUtility.h"
#include "NavigationSystem.h"
#include "MAPFPlanner.generated.h"

USTRUCT(BlueprintType)
struct FSpaceTimeNode
{
    GENERATED_BODY()

    FIntVector Loc;
    float G, H, F;
    int32 ParentIndex;
    bool bIsWait;

    FSpaceTimeNode() : Loc(0), G(0), H(0), F(0), ParentIndex(INDEX_NONE), bIsWait(false) {}
};

USTRUCT(BlueprintType)
struct FGridCell
{
    GENERATED_BODY()

    UPROPERTY()
    bool bIsWalkable;

    UPROPERTY()
    bool bIsObstacle;

    FGridCell() : bIsWalkable(true), bIsObstacle(false) {}
};

UCLASS()
class PROJECTAI_WAREHOUSE_API UMAPFPlanner : public UGameInstanceSubsystem
{
    GENERATED_BODY()

public:
    // === КОНФИГУРАЦИЯ СЕТКИ ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Grid", meta = (ClampMin = "0.5", ClampMax = "5.0"))
    float CellSizeMeters = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Grid")
    int32 GridMinX = -100;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Grid")
    int32 GridMaxX = 100;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Grid")
    int32 GridMinY = -100;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Grid")
    int32 GridMaxY = 100;

    // === ПАРАМЕТРЫ ПЛАНИРОВАНИЯ ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning", meta = (ClampMin = "0.1", ClampMax = "2.0"))
    float TimeStepSec = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning", meta = (ClampMin = "10"))
    int32 MaxTimeHorizon = 60;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning", meta = (ClampMin = "1000"))
    int32 MaxSearchNodes = 5000;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning", meta = (ClampMin = "0", ClampMax = "2"))
    int32 AgentClearance = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning")
    bool bUseLineTraceForObstacles = true;

    // === ТАБЛИЦА РЕЗЕРВИРОВАНИЯ ===
    UPROPERTY(VisibleAnywhere, Category = "Debug")
    TMap<uint64, int32> ReservationTable;

    UPROPERTY(VisibleAnywhere, Category = "Debug")
    TMap<FIntVector, FGridCell> GridCache;

    // === ПУБЛИЧНЫЙ API ===
    UFUNCTION(BlueprintCallable, Category = "MAPF")
    TArray<FVector> PlanPath(int32 AgentID, FVector StartWorld, FVector GoalWorld, float StartTime);

    UFUNCTION(BlueprintCallable, Category = "MAPF")
    bool ReservePath(int32 AgentID, const TArray<FIntVector>& Path);

    UFUNCTION(BlueprintCallable, Category = "MAPF")
    void ReleasePath(int32 AgentID, const TArray<FIntVector>& Path);

    UFUNCTION(BlueprintCallable, Category = "MAPF|Helpers")
    void InitializeGridCache();

    UFUNCTION(BlueprintCallable, Category = "MAPF|Helpers")
    FIntVector WorldToGrid(FVector Pos) const;

    UFUNCTION(BlueprintCallable, Category = "MAPF|Helpers")
    FVector GridToWorld(FIntVector GridPos) const;

    UFUNCTION(BlueprintCallable, Category = "MAPF|Utilities", meta = (WorldContext = "WorldContextObject"))
    static UMAPFPlanner* GetPlanner(const UObject* WorldContextObject);

    UFUNCTION(BlueprintCallable, Category = "MAPF|Debug")
    void DrawDebugGrid(float Duration = 10.0f, bool bDrawCells = false);

private:
    uint64 PackKey(int32 X, int32 Y, int32 T) const;
    TArray<FIntVector> GetFootprintCells(int32 X, int32 Y, int32 T) const;
    float Heuristic(const FIntVector& Current, const FIntVector& Goal) const;
    bool CheckVertexConflict(const FIntVector& Loc, int32 AgentID) const;
    bool CheckEdgeConflict(const FIntVector& From, const FIntVector& To, int32 T, int32 AgentID) const;
    bool IsCellPassable(FVector WorldPosition) const;
    bool CheckStaticObstacle(FVector Position) const;
    bool IsWithinGridBounds(const FIntVector& GridPos) const;
};