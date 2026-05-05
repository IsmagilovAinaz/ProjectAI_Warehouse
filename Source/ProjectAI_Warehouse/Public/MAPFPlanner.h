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

    UPROPERTY()
    FIntVector Loc;

    UPROPERTY()
    float G;

    UPROPERTY()
    float H;

    UPROPERTY()
    float F;

    UPROPERTY()
    int32 ParentIndex;

    UPROPERTY()
    bool bIsWait;

    UPROPERTY()
    int32 WaitCount;  // Track consecutive waits

    FSpaceTimeNode() : Loc(0), G(0), H(0), F(0), ParentIndex(INDEX_NONE), bIsWait(false), WaitCount(0) {}
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

// Reservation entry with priority
USTRUCT()
struct FReservationEntry
{
    GENERATED_BODY()

    UPROPERTY()
    int32 AgentID;

    UPROPERTY()
    float Priority;  // Lower value = higher priority

    UPROPERTY()
    float Timestamp; // When this reservation was made

    FReservationEntry() : AgentID(-1), Priority(0), Timestamp(0) {}
    FReservationEntry(int32 InAgentID, float InPriority, float InTimestamp)
        : AgentID(InAgentID), Priority(InPriority), Timestamp(InTimestamp) {
    }
};

UCLASS()
class PROJECTAI_WAREHOUSE_API UMAPFPlanner : public UGameInstanceSubsystem
{
    GENERATED_BODY()

public:
    // === CONFIGURATION ===
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

    // === PLANNING PARAMETERS ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning", meta = (ClampMin = "0.1", ClampMax = "2.0"))
    float TimeStepSec = 0.5f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning", meta = (ClampMin = "10"))
    int32 MaxTimeHorizon = 120;  // Increased for complex scenarios

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning", meta = (ClampMin = "1000"))
    int32 MaxSearchNodes = 50000;  // Increased

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning", meta = (ClampMin = "0", ClampMax = "2"))
    int32 AgentClearance = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Planning")
    bool bUseLineTraceForObstacles = true;

    // === COOPERATIVE PLANNING ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Cooperative")
    float WaitCostFactor = 1.5f;  // Penalty for waiting (encourages detours)

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Cooperative")
    float ConflictCostFactor = 10.0f;  // High penalty for conflicts

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Cooperative")
    int32 MaxConsecutiveWaits = 5;  // Prevent infinite waiting

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MAPF Config|Cooperative")
    bool bEnableCooperativePlanning = true;  // Enable priority-based negotiation

    // === RESERVATION TABLE ===
    UPROPERTY(VisibleAnywhere, Category = "Debug")
    TMap<uint64, FReservationEntry> ReservationTable;

    UPROPERTY(VisibleAnywhere, Category = "Debug")
    TMap<FIntVector, FGridCell> GridCache;

    // === PUBLIC API ===
    UFUNCTION(BlueprintCallable, Category = "MAPF")
    TArray<FVector> PlanPath(int32 AgentID, FVector StartWorld, FVector GoalWorld, float StartTime, float Priority = 0.0f);

    UFUNCTION(BlueprintCallable, Category = "MAPF")
    bool ReservePath(int32 AgentID, const TArray<FIntVector>& Path, float Priority = 0.0f);

    UFUNCTION(BlueprintCallable, Category = "MAPF")
    void ReleasePath(int32 AgentID, const TArray<FIntVector>& Path);

    UFUNCTION(BlueprintCallable, Category = "MAPF")
    bool TryStealReservation(int32 NewAgentID, int32 OldAgentID, const FIntVector& Location, float NewPriority);

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

    UFUNCTION(BlueprintCallable, Category = "MAPF|Debug")
    void DebugPrintReservations() const;

private:
    uint64 PackKey(int32 X, int32 Y, int32 T) const;
    void UnpackKey(uint64 Key, int32& OutX, int32& OutY, int32& OutT) const;
    TArray<FIntVector> GetFootprintCells(int32 X, int32 Y, int32 T) const;
    float Heuristic(const FIntVector& Current, const FIntVector& Goal) const;
    bool CheckVertexConflict(const FIntVector& Loc, int32 AgentID, float Priority) const;
    bool CheckEdgeConflict(const FIntVector& From, const FIntVector& To, int32 T, int32 AgentID, float Priority) const;
    bool IsCellPassable(FVector WorldPosition) const;
    bool CheckStaticObstacle(FVector Position) const;
    bool IsWithinGridBounds(const FIntVector& GridPos) const;
    float CalculateConflictPenalty(const FIntVector& Loc, int32 AgentID, float Priority) const;
    void CleanupStaleReservations(float CurrentTime);
};