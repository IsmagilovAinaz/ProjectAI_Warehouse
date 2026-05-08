#include "SimulationDataCollector.h"
#include "Misc/FileHelper.h"
#include "Engine/GameInstance.h"
#include "Engine/World.h"

USimulationDataCollector* USimulationDataCollector::GetCollector(const UObject* WorldContextObject)
{
    if (!WorldContextObject) return nullptr;
    UWorld* World = WorldContextObject->GetWorld();
    if (!World) return nullptr;
    UGameInstance* GI = World->GetGameInstance();
    if (!GI) return nullptr;
    return GI->GetSubsystem<USimulationDataCollector>();
}

void USimulationDataCollector::RegisterTask(const FTaskMetrics& Task)
{
    ActiveTasks.Add(Task.TaskID, Task);
}

void USimulationDataCollector::LogTaskCompleted(const FString& TaskID, float CompletionTime)
{
    if (FTaskMetrics* Task = ActiveTasks.Find(TaskID))
    {
        Task->TimeCompleted = CompletionTime;
        CompletedTasks.Add(*Task);
        ActiveTasks.Remove(TaskID);
    }
}

void USimulationDataCollector::UpdateAgentMetrics(int32 AgentID, const FAgentMetrics& Metrics)
{
    AgentMetricsMap.Add(AgentID, Metrics);
}

void USimulationDataCollector::ExportToCSV(const FString& FilePath)
{
    FString CSV;

    // Task metrics
    CSV += TEXT("=== TASK METRICS ===\n");
    CSV += TEXT("TaskID,TimeReceived,AssignedAgent,AuctionBid,PathLength,CompletionTime\n");
    for (const FTaskMetrics& T : CompletedTasks)
    {
        CSV += FString::Printf(TEXT("%s,%.2f,%d,%.2f,%d,%.2f\n"),
            *T.TaskID, T.TimeReceived, T.AssignedAgentID,
            T.AuctionBid, T.PathLength, T.TimeCompleted);
    }

    CSV += TEXT("\n=== AGENT METRICS ===\n");
    CSV += TEXT("AgentID,TasksCompleted,TotalDistance,IdleTime,MovingTime,WaitingTime,ReplanCount\n");
    for (const auto& Pair : AgentMetricsMap)
    {
        const FAgentMetrics& M = Pair.Value;
        CSV += FString::Printf(TEXT("%d,%d,%.1f,%.2f,%.2f,%.2f,%d\n"),
            M.AgentID, M.TasksCompleted, M.TotalDistance,
            M.IdleTime, M.MovingTime, M.WaitingTime, M.ReplanCount);
    }

    CSV += TEXT("\n=== GLOBAL METRICS ===\n");
    CSV += FString::Printf(TEXT("TotalTasks,%d\n"), CompletedTasks.Num());
    CSV += FString::Printf(TEXT("TotalConflicts,%d\n"), TotalConflictsDetected);
    CSV += FString::Printf(TEXT("TotalReplans,%d\n"), TotalReplans);

    if (CompletedTasks.Num() > 0)
    {
        float AvgTime = 0;
        for (const auto& T : CompletedTasks) AvgTime += T.TimeCompleted;
        AvgTime /= CompletedTasks.Num();
        CSV += FString::Printf(TEXT("AvgCompletionTime,%.2f\n"), AvgTime);
    }

    FFileHelper::SaveStringToFile(CSV, *FilePath);
    UE_LOG(LogTemp, Log, TEXT("Simulation data exported to: %s"), *FilePath);
}