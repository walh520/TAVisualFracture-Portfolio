#include "TAVisualFractureComponentDetails.h"

#include "TAVisualFractureBakeService.h"
#include "TAVisualFractureComponent.h"
#include "TAVisualFractureLandscapeService.h"
#include "TAVisualFracturePreviewManager.h"

#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "Framework/Notifications/NotificationManager.h"
#include "Widgets/Input/SButton.h"
#include "Widgets/Layout/SUniformGridPanel.h"
#include "Widgets/Notifications/SProgressBar.h"
#include "Widgets/Notifications/SNotificationList.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "TAVisualFractureComponentDetails"

namespace
{
void NotifyFailure(const FString& Error)
{
    if (Error.IsEmpty()) return;
    FNotificationInfo Info(FText::FromString(Error));
    Info.ExpireDuration = 6.0f;
    Info.bFireAndForget = true;
    if (TSharedPtr<SNotificationItem> Item = FSlateNotificationManager::Get().AddNotification(Info))
    {
        Item->SetCompletionState(SNotificationItem::CS_Fail);
    }
}
}

TSharedRef<IDetailCustomization> FTAVisualFractureComponentDetails::MakeInstance()
{
    return MakeShared<FTAVisualFractureComponentDetails>();
}

void FTAVisualFractureComponentDetails::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
    TArray<TWeakObjectPtr<UObject>> Objects;
    DetailBuilder.GetObjectsBeingCustomized(Objects);
    if (Objects.Num() != 1)
    {
        return;
    }
    Component = Cast<UTAVisualFractureComponent>(Objects[0].Get());
    if (!Component.IsValid())
    {
        return;
    }

    IDetailCategoryBuilder& Actions = DetailBuilder.EditCategory(TEXT("TA Visual Fracture Actions"),
        LOCTEXT("ActionsCategory", "TA Visual Fracture 快速预览"), ECategoryPriority::Important);
    Actions.AddCustomRow(LOCTEXT("BakeRow", "Bake Cancel Capture"))
    .WholeRowContent()
    [
        SNew(SUniformGridPanel)
        .SlotPadding(FMargin(2.0f))
        + SUniformGridPanel::Slot(0, 0)
        [
            SNew(SButton)
            .Text_Lambda([Weak = Component]()
            {
                return Weak.IsValid() && Weak->bBakeInProgress ? LOCTEXT("CancelBake", "取消烘焙") : LOCTEXT("Bake", "烘焙");
            })
            .ToolTipText(LOCTEXT("BakeTooltip", "根据目标网格和烘焙参数生成碎块资产；烘焙中再次点击可取消。"))
            .OnClicked_Lambda([Weak = Component]()
            {
                if (UTAVisualFractureComponent* C = Weak.Get())
                {
                    C->LastPreviewStatus = TEXT("Preview reset for bake.");
                    FTAVisualFracturePreviewManager::Get().Reset(C);
                    if (C->bBakeInProgress) FTAVisualFractureBakeService::Get().CancelBake(C);
                    else FTAVisualFractureBakeService::Get().StartBake(C);
                }
                return FReply::Handled();
            })
        ]
        + SUniformGridPanel::Slot(1, 0)
        [
            SNew(SButton)
            .Text(LOCTEXT("Capture", "捕获 Landscape"))
            .ToolTipText(LOCTEXT("CaptureTooltip", "采样目标周围的 Landscape 高度快照，供碎块落地使用。"))
            .OnClicked_Lambda([Weak = Component]()
            {
                if (UTAVisualFractureComponent* C = Weak.Get())
                {
                    C->LastPreviewStatus = TEXT("Preview reset for Landscape capture.");
                    FTAVisualFracturePreviewManager::Get().Reset(C);
                    if (!FTAVisualFractureLandscapeService::Capture(C)) NotifyFailure(C->LastCaptureStatus);
                }
                return FReply::Handled();
            })
        ]
    ];

    Actions.AddCustomRow(LOCTEXT("ProgressRow", "Bake Progress"))
    .WholeRowContent()
    [
        SNew(SProgressBar)
        .Percent_Lambda([Weak = Component]() -> TOptional<float>
        {
            return Weak.IsValid() ? TOptional<float>(Weak->BakeProgress) : TOptional<float>();
        })
    ];

    Actions.AddCustomRow(LOCTEXT("PresetRow", "Top Side Center"))
    .WholeRowContent()
    [
        SNew(SUniformGridPanel)
        .SlotPadding(FMargin(2.0f))
        + SUniformGridPanel::Slot(0, 0)
        [
            SNew(SButton).Text(LOCTEXT("Top", "顶部"))
            .ToolTipText(LOCTEXT("TopTooltip", "从物体上方寻找首个表面点，并载入顶部冲击参数。"))
            .OnClicked_Lambda([Weak = Component]()
            {
                FString Error;
                if (UTAVisualFractureComponent* C = Weak.Get())
                    if (!FTAVisualFracturePreviewManager::Get().ApplyPreset(C, static_cast<uint8>(ETVFImpactLocationPreset::Top), Error)) NotifyFailure(Error);
                return FReply::Handled();
            })
        ]
        + SUniformGridPanel::Slot(1, 0)
        [
            SNew(SButton).Text(LOCTEXT("Side", "侧面中部"))
            .ToolTipText(LOCTEXT("SideTooltip", "从所选侧轴向内寻找表面点，并载入侧面冲击参数。"))
            .OnClicked_Lambda([Weak = Component]()
            {
                FString Error;
                if (UTAVisualFractureComponent* C = Weak.Get())
                    if (!FTAVisualFracturePreviewManager::Get().ApplyPreset(C, static_cast<uint8>(ETVFImpactLocationPreset::SideMiddle), Error)) NotifyFailure(Error);
                return FReply::Handled();
            })
        ]
        + SUniformGridPanel::Slot(2, 0)
        [
            SNew(SButton).Text(LOCTEXT("Center", "中心内部"))
            .ToolTipText(LOCTEXT("CenterTooltip", "使用烘焙时记录的有效内部点，并载入中心冲击参数。"))
            .OnClicked_Lambda([Weak = Component]()
            {
                FString Error;
                if (UTAVisualFractureComponent* C = Weak.Get())
                    if (!FTAVisualFracturePreviewManager::Get().ApplyPreset(C, static_cast<uint8>(ETVFImpactLocationPreset::CenterInterior), Error)) NotifyFailure(Error);
                return FReply::Handled();
            })
        ]
    ];

    Actions.AddCustomRow(LOCTEXT("PreviewRow", "Break Reset"))
    .WholeRowContent()
    [
        SNew(SUniformGridPanel)
        .SlotPadding(FMargin(2.0f))
        + SUniformGridPanel::Slot(0, 0)
        [
            SNew(SButton).Text(LOCTEXT("Break", "破碎"))
            .ToolTipText(LOCTEXT("BreakTooltip", "提交一次冲击事件并开始关卡视口预览；可再次点击累积 Bond 损伤。"))
            .OnClicked_Lambda([Weak = Component]()
            {
                FString Error;
                if (UTAVisualFractureComponent* C = Weak.Get())
                    if (!FTAVisualFracturePreviewManager::Get().Break(C, Error)) NotifyFailure(Error);
                return FReply::Handled();
            })
        ]
        + SUniformGridPanel::Slot(1, 0)
        [
            SNew(SButton).Text(LOCTEXT("Reset", "重置"))
            .ToolTipText(LOCTEXT("ResetTooltip", "结束预览、销毁临时碎块并恢复原始网格。"))
            .OnClicked_Lambda([Weak = Component]()
            {
                if (Weak.IsValid()) Weak->LastPreviewStatus = TEXT("Preview reset by user; source mesh restored.");
                FTAVisualFracturePreviewManager::Get().Reset(Weak.Get());
                return FReply::Handled();
            })
        ]
    ];

    Actions.AddCustomRow(LOCTEXT("CollisionActions", "Prepare Collision Show Proxies"))
    .WholeRowContent()
    [
        SNew(SUniformGridPanel).SlotPadding(FMargin(2.0f))
        + SUniformGridPanel::Slot(0, 0)
        [
            SNew(SButton).Text(LOCTEXT("PrepareCollision", "准备碰撞／冻结原位"))
            .IsEnabled_Lambda([Weak = Component]() { return Weak.IsValid() && !Weak->bBakeInProgress; })
            .OnClicked_Lambda([Weak = Component]()
            {
                FString Error;
                if (UTAVisualFractureComponent* C = Weak.Get())
                    if (!FTAVisualFracturePreviewManager::Get().PrepareCollision(C, Error)) NotifyFailure(Error);
                return FReply::Handled();
            })
        ]
        + SUniformGridPanel::Slot(1, 0)
        [
            SNew(SButton).Text(LOCTEXT("ShowCollision", "显示／隐藏代理"))
            .OnClicked_Lambda([Weak = Component]()
            {
                FString Error;
                if (UTAVisualFractureComponent* C = Weak.Get())
                    if (!FTAVisualFracturePreviewManager::Get().ToggleCollisionProxies(C, Error)) NotifyFailure(Error);
                return FReply::Handled();
            })
        ]
    ];

    Actions.AddCustomRow(LOCTEXT("StatusRow", "Status"))
    .WholeRowContent()
    [
        SNew(STextBlock)
        .AutoWrapText(true)
        .Text_Lambda([Weak = Component]()
        {
            if (!Weak.IsValid()) return FText::GetEmpty();
            return FText::FromString(FString::Printf(TEXT("Bake: %s\nLandscape: %s\nPreview: %s [Detached=%d BrokenBonds=%d]\nCollision: %s"),
                *Weak->LastBakeStatus, *Weak->LastCaptureStatus, *Weak->LastPreviewStatus,
                Weak->PreviewDetachedChunks, Weak->PreviewBrokenBonds, *Weak->LastCollisionStatus));
        })
    ];
}

#undef LOCTEXT_NAMESPACE
