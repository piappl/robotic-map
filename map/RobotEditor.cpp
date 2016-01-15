#include "RobotEditor.h"

RobotEditor::RobotEditor(QQuickItem *parent) : QQuickItem(parent) {}

void RobotEditor::orientate() { emit orientateToCrosshair(); }
void RobotEditor::position() { emit placeAtCrosshair(); }
void RobotEditor::finalizeEdit(bool persist) { emit finalizePositionEdit(persist); }
void RobotEditor::startEdit() { emit startPositionEdit(); }
void RobotEditor::orientationEditChanged(bool enable) { emit orientationEdit(enable); }
