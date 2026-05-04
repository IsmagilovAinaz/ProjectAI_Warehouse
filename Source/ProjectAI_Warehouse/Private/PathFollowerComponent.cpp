#include "PathFollowerComponent.h"
#include "GameFramework/Character.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "MAPFPlanner.h"
#include "DrawDebugHelpers.h"

UPathFollowerComponent::UPathFollowerComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    bFinished = false;
    bReleased = false;
}

void UPathFollowerComponent::SetPath(const TArray<FVector>& Path)
{
    ClearDebugPath();

    Waypoints = Path;
    CurrentIndex = 0;
    bFinished = (Waypoints.Num() == 0);
    bReleased = false;

    // Логирование для диссертации (§3.4)
    if (Waypoints.Num() > 0)
    {
        UE_LOG(LogTemp, Log, TEXT("PathFollower: Agent %d received path with %d points"),
            AgentID, Waypoints.Num());
        UE_LOG(LogTemp, Log, TEXT("  Start: (%.1f, %.1f) -> Goal: (%.1f, %.1f)"),
            Waypoints[0].X, Waypoints[0].Y,
            Waypoints.Last().X, Waypoints.Last().Y);
    }

    if (bDrawDebugPath)
    {
        DrawDebugPath();
    }
}

void UPathFollowerComponent::DrawDebugPath()
{
    if (Waypoints.Num() < 2 || !bDrawDebugPath) return;

    UWorld* World = GetWorld();
    if (!World) return;

    // Рисуем линии маршрута
    for (int32 i = 0; i < Waypoints.Num() - 1; i++)
    {
        FVector Start = Waypoints[i];
        FVector End = Waypoints[i + 1];
        Start.Z += 15.0f;
        End.Z += 15.0f;

        FColor Color = (i == CurrentIndex) ? FColor::Yellow : FColor::Green;
        float Thickness = (i == CurrentIndex) ? 3.0f : 1.5f;

        DrawDebugLine(
            World, Start, End, Color,
            false, DebugDrawDuration, 0, Thickness
        );
    }

    // Финальная точка
    FVector Last = Waypoints.Last();
    Last.Z += 15.0f;
    DrawDebugSphere(World, Last, 12.0f, 8, FColor::Blue, false, DebugDrawDuration);
}

void UPathFollowerComponent::ClearDebugPath()
{
    if (UWorld* World = GetWorld())
    {
        FlushPersistentDebugLines(World);
    }
}

void UPathFollowerComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (bFinished || Waypoints.Num() == 0)
    {
        if (bFinished && !bReleased)
        {
            bReleased = true;
            // Освобождаем резервации пути
            UMAPFPlanner* Planner = UMAPFPlanner::GetPlanner(GetOwner());
            if (Planner)
            {
                TArray<FIntVector> GridPath;
                GridPath.Reserve(Waypoints.Num());
                for (const FVector& P : Waypoints)
                {
                    GridPath.Add(Planner->WorldToGrid(P));
                }
                Planner->ReleasePath(AgentID, GridPath);
            }
            OnPathFinished();
        }
        return;
    }

    ACharacter* Char = Cast<ACharacter>(GetOwner());
    if (!Char) return;

    UCharacterMovementComponent* CMC = Char->GetCharacterMovement();
    if (!CMC) return;

    // 2D-движение: отключаем влияние гравитации
    CMC->GravityScale = 0.f;

    FVector CurrentPos = GetOwner()->GetActorLocation();
    FVector TargetPos = Waypoints[CurrentIndex];
    CurrentPos.Z = TargetPos.Z;

    FVector Dir = TargetPos - CurrentPos;
    float Dist = Dir.Size();

    // Переключение на следующую точку
    if (Dist < ArrivalThreshold)
    {
        CurrentIndex++;
        if (CurrentIndex >= Waypoints.Num())
        {
            bFinished = true;
            CMC->Velocity = FVector::ZeroVector;
            CMC->StopActiveMovement();
            return;
        }
        // Мгновенное обновление цели
        TargetPos = Waypoints[CurrentIndex];
        CurrentPos = GetOwner()->GetActorLocation();
        CurrentPos.Z = TargetPos.Z;
        Dir = TargetPos - CurrentPos;
        Dist = Dir.Size();
    }

    // Поворот и движение
    if (Dist > 1.0f)
    {
        Dir.Normalize();

        // Плавный поворот
        FRotator TargetRot = Dir.Rotation();
        TargetRot.Pitch = 0.f;
        TargetRot.Roll = 0.f;

        FRotator NewRot = FMath::RInterpConstantTo(
            GetOwner()->GetActorRotation(),
            TargetRot,
            DeltaTime,
            RotationSpeed
        );
        GetOwner()->SetActorRotation(NewRot);

        // Прямое управление скоростью (исключает инерционные артефакты)
        CMC->Velocity = Dir * Speed;
    }
    else
    {
        CMC->Velocity = FVector::ZeroVector;
    }

    // Обновление отладочной визуализации
    if (bDrawDebugPath)
    {
        DrawDebugPath();
    }
}

void UPathFollowerComponent::OnPathFinished()
{
    // Пустая реализация — можно переопределить в Blueprint
    UE_LOG(LogTemp, Verbose, TEXT("PathFollower: Agent %d completed path"), AgentID);
}