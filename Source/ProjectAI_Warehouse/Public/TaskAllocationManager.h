#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "TaskAuctionManager.h"  // дл€ FAuctionResult
#include "WarehouseTypes.h"
#include "TaskAllocationManager.generated.h"

class ATaskAuctionManager;

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class PROJECTAI_WAREHOUSE_API UTaskAllocationManager : public UActorComponent
{
    GENERATED_BODY()

public:
    UTaskAllocationManager();

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Allocation")
    EAllocationStrategy CurrentStrategy = EAllocationStrategy::Auction;

    UFUNCTION(BlueprintCallable, Category = "Allocation")
    void SetStrategy(EAllocationStrategy NewStrategy);

    UFUNCTION(BlueprintCallable, Category = "Allocation")
    EAllocationStrategy GetStrategy() const { return CurrentStrategy; }

    // √лавный метод Ч возвращает FAuctionResult как и аукцион
    UFUNCTION(BlueprintCallable, Category = "Allocation")
    FAuctionResult AllocateTask(
        const FString& TaskID,
        FVector TaskLocation,
        float TaskPriority,
        float PayloadMass,
        const TMap<int32, float>& EstimatedTravelTimes
    );

    static UTaskAllocationManager* GetManager(UObject* WorldContext);

private:
    ATaskAuctionManager* FindAuctionManager() const;

    FAuctionResult GreedyAllocation(
        const FString& TaskID,
        FVector TaskLocation,
        float PayloadMass
    ) const;

    FAuctionResult AuctionAllocation(
        const FString& TaskID,
        FVector TaskLocation,
        float TaskPriority,
        float PayloadMass,
        const TMap<int32, float>& EstimatedTravelTimes
    );
};