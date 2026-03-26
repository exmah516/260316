// ============================================================
// main.cpp - Qt 应用入口
// ============================================================

#include <QApplication>
#include "SharedState.h"
#include "MainWindow.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    app.setApplicationName("CatheterRobotUI");

    SharedState shared;
    MainWindow window(shared);
    window.show();

    return app.exec();
}
