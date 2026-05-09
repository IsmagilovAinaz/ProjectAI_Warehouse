#pragma once

#include "CoreMinimal.h"
#include "Subsystems/GameInstanceSubsystem.h"
#include "SimulationDataCollector.generated.h"

USTRUCT(BlueprintType)
struct FTaskMetrics
{
    GENERATED_BODY()

    UPROPERTY() FString TaskID;
    UPROPERTY() float TimeReceived = 0;
    UPROPERTY() float TimeAssigned = 0;
    UPROPERTY() float TimeStarted = 0;
    UPROPERTY() float TimeCompleted = 0;
    UPROPERTY() int32 AssignedAgentID = -1;
    UPROPERTY() float AuctionBid = 0;
    UPROPERTY() int32 PathLength = 0;
};

USTRUCT(BlueprintType)
struct FAgentMetrics
{
    GENERATED_BODY()

    UPROPERTY() int32 AgentID = -1;
    UPROPERTY() float TotalDistance = 0;
    UPROPERTY() float IdleTime = 0;
    UPROPERTY() float MovingTime = 0;
    UPROPERTY() float WaitingTime = 0;
    UPROPERTY() int32 TasksCompleted = 0;
    UPROPERTY() int32 ReplanCount = 0;
};

UCLASS()
class PROJECTAI_WAREHOUSE_API USimulationDataCollector : public UGameInstanceSubsystem
{
    GENERATED_BODY()

public:
    UFUNCTION(BlueprintCallable, Category = "Simulation Data")
    void RegisterTask(const FTaskMetrics& Task);

    UFUNCTION(BlueprintCallable, Category = "Simulation Data")
    void UpdateAgentMetrics(int32 AgentID, const FAgentMetrics& Metrics);

    UFUNCTION(BlueprintCallable, Category = "Simulation Data")
    void LogTaskCompleted(const FString& TaskID, float CompletionTime);

    UFUNCTION(BlueprintCallable, Category = "Simulation Data")
    void ExportToCSV(const FString& FilePath);

    UFUNCTION(BlueprintCallable, Category = "Simulation Data")
    static USimulationDataCollector* GetCollector(const UObject* WorldContextObject);

    UPROPERTY(BlueprintReadOnly, Category = "Simulation Data")
    TArray<FTaskMetrics> CompletedTasks;

    UPROPERTY(BlueprintReadOnly, Category = "Simulation Data")
    TMap<int32, FAgentMetrics> AgentMetricsMap;

    UPROPERTY(BlueprintReadWrite, Category = "Simulation Data")
    int32 TotalConflictsDetected = 0;

    UPROPERTY(BlueprintReadWrite, Category = "Simulation Data")
    int32 TotalReplans = 0;

    UPROPERTY(BlueprintReadWrite, Category = "Simulation Data")
    float SimulationStartTime = 0;

    UPROPERTY()
    TMap<FString, FTaskMetrics> ActiveTasks;

private:
    
};