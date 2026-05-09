#include "RobotStateVisualizer.h"
#include "Components/WidgetComponent.h"
#include "Blueprint/UserWidget.h"
#include "Components/TextBlock.h"
#include "Components/Image.h"
#include "Engine/World.h"
#include "Blueprint/WidgetTree.h"
#include "Components/EditableTextBox.h"

URobotStateVisualizer::URobotStateVisualizer()
{
    PrimaryComponentTick.bCanEverTick = false;
}

void URobotStateVisualizer::BeginPlay()
{
    Super::BeginPlay();

    if (bShowWidget)
    {
        CreateStatusWidget();
    }
}

void URobotStateVisualizer::CreateStatusWidget()
{
    AActor* Owner = GetOwner();
    if (!Owner) return;

    // Создаем WidgetComponent
    StatusWidgetComponent = NewObject<UWidgetComponent>(Owner, UWidgetComponent::StaticClass());
    if (!StatusWidgetComponent) return;

    StatusWidgetComponent->RegisterComponent();
    StatusWidgetComponent->AttachToComponent(
        Owner->GetRootComponent(),
        FAttachmentTransformRules::SnapToTargetNotIncludingScale
    );
    StatusWidgetComponent->SetRelativeLocation(WidgetOffset);
    StatusWidgetComponent->SetWidgetSpace(EWidgetSpace::Screen); // Всегда лицом к камере
    StatusWidgetComponent->SetDrawSize(FVector2D(220, 90));
    StatusWidgetComponent->SetTwoSided(true);
    StatusWidgetComponent->SetHiddenInGame(false);

    // Устанавливаем класс виджета
    if (StatusWidgetClass)
    {
        StatusWidgetComponent->SetWidgetClass(StatusWidgetClass);
        StatusWidgetInstance = StatusWidgetComponent->GetWidget();
    }

    // Начальное состояние
    UpdateStatusWidget();

    if (bDebugLog)
    {
        UE_LOG(LogTemp, Log, TEXT("RobotStateVisualizer: Widget created for %s at height %.1f"),
            *Owner->GetName(), WidgetOffset.Z);
    }
}

void URobotStateVisualizer::SetAgentName(const FString& Name)
{
    AgentName = Name;
    UpdateStatusWidget();
}

void URobotStateVisualizer::SetVisualState(ERobotVisualState NewState)
{
    CurrentState = NewState;
    UpdateStatusWidget();

    if (bDebugLog)
    {
        FRobotStateInfo Info = GetStateInfo(NewState);
        UE_LOG(LogTemp, Log, TEXT("RobotStateVisualizer: %s -> %s"),
            *AgentName, *Info.StateText.ToString());
    }
}

FRobotStateInfo URobotStateVisualizer::GetStateInfo(ERobotVisualState State) const
{
    FRobotStateInfo Info;

    switch (State)
    {
    case ERobotVisualState::Idle:
        Info.StateText = FText::FromString(TEXT("Ожидание задачи"));
        Info.StateColor = FLinearColor(0.7f, 0.7f, 0.7f); // Серый
        break;

    case ERobotVisualState::MovingToPickup:
        Info.StateText = FText::FromString(TEXT("К паллете"));
        Info.StateColor = FLinearColor(0.3f, 0.6f, 1.0f); // Синий
        break;

    case ERobotVisualState::GrabbingPallet:
        Info.StateText = FText::FromString(TEXT("Захват груза"));
        Info.StateColor = FLinearColor(0.2f, 0.8f, 1.0f); // Голубой
        break;

    case ERobotVisualState::MovingToDropoff:
        Info.StateText = FText::FromString(TEXT("К стеллажу"));
        Info.StateColor = FLinearColor(0.1f, 0.8f, 0.3f); // Зеленый
        break;

    case ERobotVisualState::PlacingPallet:
        Info.StateText = FText::FromString(TEXT("Размещение"));
        Info.StateColor = FLinearColor(0.2f, 0.9f, 0.5f); // Светло-зеленый
        break;

    case ERobotVisualState::Waiting:
        Info.StateText = FText::FromString(TEXT("Ожидание (MAPF)"));
        Info.StateColor = FLinearColor(1.0f, 0.8f, 0.0f); // Желтый
        break;

    case ERobotVisualState::Conflict:
        Info.StateText = FText::FromString(TEXT("Конфликт"));
        Info.StateColor = FLinearColor(1.0f, 0.2f, 0.2f); // Красный
        break;

    case ERobotVisualState::Charging:
        Info.StateText = FText::FromString(TEXT("Зарядка"));
        Info.StateColor = FLinearColor(1.0f, 0.5f, 0.0f); // Оранжевый
        break;
    }

    return Info;
}

void URobotStateVisualizer::UpdateStatusWidget()
{
    if (!StatusWidgetInstance) return;

    FRobotStateInfo Info = GetStateInfo(CurrentState);

    UEditableTextBox* StatusBox = Cast<UEditableTextBox>(
        StatusWidgetInstance->GetWidgetFromName(FName(TEXT("StatusValue")))
    );

    if (StatusBox)
    {
        // Устанавливаем текст
        StatusBox->SetText(Info.StateText);

        // Устанавливаем цвет текста через WidgetStyle
        FEditableTextBoxStyle Style = StatusBox->WidgetStyle;
        Style.ForegroundColor = Info.StateColor;
        StatusBox->WidgetStyle = Style;

        UE_LOG(LogTemp, Log, TEXT("StatusWidget: Updated to '%s'"),
            *Info.StateText.ToString());
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("StatusWidget: StatusValue not found!"));
    }
}


void URobotStateVisualizer::UpdateAgentState(AActor* Owner, ERobotVisualState NewState)
{
    if (!Owner) return;

    URobotStateVisualizer* Viz = Owner->FindComponentByClass<URobotStateVisualizer>();
    if (!Viz) return;

    Viz->SetVisualState(NewState);
}

void URobotStateVisualizer::SetHighlighted(bool bHighlighted)
{
    if (bHighlighted)
    {
        // Временно показать состояние "выбран"
        FRobotStateInfo Info;
        Info.StateText = FText::FromString(TEXT("ВЫБРАН"));
        Info.StateColor = HighlightColor;

        if (StatusWidgetInstance)
        {
            UEditableTextBox* StatusBox = Cast<UEditableTextBox>(
                StatusWidgetInstance->GetWidgetFromName(FName(TEXT("StatusValue")))
            );
            if (StatusBox)
            {
                StatusBox->SetText(Info.StateText);
                // Мигание или постоянный цвет
            }
        }
    }
    else
    {
        // Восстановить статус
        UpdateStatusWidget();
    }
}