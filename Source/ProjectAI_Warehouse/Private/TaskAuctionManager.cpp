#include "TaskAuctionManager.h"
#include "RobotAuctionComponent.h"
#include "Engine/World.h"
#include "TimerManager.h"

ATaskAuctionManager::ATaskAuctionManager()
{
    PrimaryActorTick.bCanEverTick = false;
}

void ATaskAuctionManager::RegisterRobot(URobotAuctionComponent* Robot)
{
    if (Robot && !RegisteredRobots.Contains(Robot))
    {
        RegisteredRobots.Add(Robot);
        UE_LOG(LogTemp, Log, TEXT("Auction Manager: Registered Robot %d (total: %d)"),
            Robot->RobotID, RegisteredRobots.Num());
    }
}

void ATaskAuctionManager::UnregisterRobot(URobotAuctionComponent* Robot)
{
    if (Robot)
    {
        RegisteredRobots.Remove(Robot);
        UE_LOG(LogTemp, Log, TEXT("Auction Manager: Unregistered Robot %d (total: %d)"),
            Robot->RobotID, RegisteredRobots.Num());
    }
}

TArray<URobotAuctionComponent*> ATaskAuctionManager::GetFeasibleRobots(float PayloadMass) const
{
    TArray<URobotAuctionComponent*> Feasible;
    for (URobotAuctionComponent* Robot : RegisteredRobots)
    {
        if (Robot && Robot->IsFeasible(PayloadMass))
        {
            Feasible.Add(Robot);
        }
    }
    return Feasible;
}

FAuctionResult ATaskAuctionManager::RunAuction(
    const FString& TaskID,
    FVector TaskLocation,
    float TaskPriority,
    float PayloadMass,
    const TMap<int32, float>& EstimatedTravelTimes)
{
    return RunAuctionInternal(TaskID, TaskLocation, TaskPriority, PayloadMass, EstimatedTravelTimes);
}

FAuctionResult ATaskAuctionManager::RunAuctionInternal(
    const FString& TaskID,
    FVector TaskLocation,
    float TaskPriority,
    float PayloadMass,
    const TMap<int32, float>& EstimatedTravelTimes)
{
    FAuctionResult Result;
    Result.TaskID = TaskID;
    Result.bSuccess = false;

    if (bDebugAuction)
    {
        UE_LOG(LogTemp, Warning, TEXT("=== AUCTION: Task %s ==="), *TaskID);
    }

    TArray<URobotAuctionComponent*> FeasibleRobots = GetFeasibleRobots(PayloadMass);

    if (FeasibleRobots.Num() == 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Auction: NO feasible robots for task %s!"), *TaskID);
        return Result;
    }

    // Сбор ставок
    for (URobotAuctionComponent* Robot : FeasibleRobots)
    {
        // Получаем предварительно вычисленное время пути для этого робота
        float TravelTime = EstimatedTravelTimes.FindRef(Robot->RobotID);
        if (TravelTime <= 0.0f)
        {
            // Fallback: быстрая оценка
            TravelTime = Robot->EstimateTravelTime(Robot->GetCurrentLocation(), TaskLocation);
        }

        FAuctionBid Bid = Robot->CalculateBid(TaskID, TaskLocation, TaskPriority, PayloadMass, TravelTime);
        Result.AllBids.Add(Bid);

        if (bDebugAuction)
        {
            UE_LOG(LogTemp, Log, TEXT("  Robot %d: bid=%.4f travel=%.1fs"),
                Bid.RobotID, Bid.BidValue, Bid.TravelTime);
        }
    }

    // Сортировка и выбор победителя
    Result.AllBids.Sort([](const FAuctionBid& A, const FAuctionBid& B) {
        return A.BidValue < B.BidValue;
        });

    if (Result.AllBids.Num() > 0 && Result.AllBids[0].BidValue < FLT_MAX)
    {
        Result.WinnerRobotID = Result.AllBids[0].RobotID;
        Result.WinningBid = Result.AllBids[0].BidValue;
        Result.bSuccess = true;

        if (bDebugAuction)
        {
            UE_LOG(LogTemp, Warning, TEXT("WINNER: Robot %d (bid=%.4f)"),
                Result.WinnerRobotID, Result.WinningBid);
        }
    }

    // Оповещение через делегат
    OnAuctionCompleted.Broadcast(Result);

    return Result;
}

URobotAuctionComponent* ATaskAuctionManager::GetRobotByID(int32 RobotID) const
{
    for (URobotAuctionComponent* Robot : RegisteredRobots)
    {
        if (Robot && Robot->RobotID == RobotID) return Robot;
    }
    return nullptr;
}