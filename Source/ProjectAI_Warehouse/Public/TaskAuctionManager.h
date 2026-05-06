#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "AuctionTypes.h"
#include "TaskAuctionManager.generated.h"

class URobotAuctionComponent;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnAuctionCompleted, const FAuctionResult&, Result);

UCLASS()
class PROJECTAI_WAREHOUSE_API ATaskAuctionManager : public AActor
{
    GENERATED_BODY()

public:
    ATaskAuctionManager();

    // === КОНФИГУРАЦИЯ ===
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    float AuctionTimeout = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Auction|Config")
    bool bDebugAuction = false;

    // === РОБОТЫ ===
    UPROPERTY(VisibleAnywhere, BlueprintReadWrite, Category = "Auction|Robots")
    TArray<URobotAuctionComponent*> RegisteredRobots;

    // === СОБЫТИЯ ===
    UPROPERTY(BlueprintAssignable, Category = "Auction|Events")
    FOnAuctionCompleted OnAuctionCompleted;

    // === МЕТОДЫ ===
    UFUNCTION(BlueprintCallable, Category = "Auction")
    void RegisterRobot(URobotAuctionComponent* Robot);

    UFUNCTION(BlueprintCallable, Category = "Auction")
    void UnregisterRobot(URobotAuctionComponent* Robot);

    /**
     * Запуск аукциона для задачи
     * @param TaskID - ID задачи
     * @param TaskLocation - позиция задачи
     * @param TaskPriority - приоритет
     * @param PayloadMass - масса груза
     * @param EstimatedTravelTimes - Map: RobotID -> EstimatedTravelTime (вычисляется вызывающим кодом)
     */
    UFUNCTION(BlueprintCallable, Category = "Auction")
    FAuctionResult RunAuction(
        const FString& TaskID,
        FVector TaskLocation,
        float TaskPriority,
        float PayloadMass,
        const TMap<int32, float>& EstimatedTravelTimes
    );

    UFUNCTION(BlueprintCallable, Category = "Auction")
    URobotAuctionComponent* GetRobotByID(int32 RobotID) const;

    UFUNCTION(BlueprintCallable, Category = "Auction")
    TArray<URobotAuctionComponent*> GetFeasibleRobots(float PayloadMass) const;

private:
    FAuctionResult RunAuctionInternal(
        const FString& TaskID,
        FVector TaskLocation,
        float TaskPriority,
        float PayloadMass,
        const TMap<int32, float>& EstimatedTravelTimes
    );
};