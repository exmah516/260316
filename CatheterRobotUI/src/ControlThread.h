#pragma once
// ============================================================
// ControlThread.h
// 在独立线程中运行 ControlEngine::tick()
// ============================================================

#include <QThread>
#include "ControlEngine.h"
#include "SharedState.h"

class ControlThread : public QThread
{
    Q_OBJECT

public:
    explicit ControlThread(SharedState& shared, QObject* parent = nullptr)
        : QThread(parent), shared_(shared) {}

    void requestStop() { stop_requested_ = true; }

signals:
    void initFinished(bool success, const QString& message);

protected:
    void run() override
    {
        ControlEngine engine(shared_);

        bool ok = engine.init();
        emit initFinished(ok, ok ? "控制引擎启动成功" : "控制引擎启动失败");

        if (!ok) return;

        while (!stop_requested_)
        {
            if (!engine.tick())
                break;

            // 控制周期 ~10ms (与原 main.cpp 的 Sleep(10) 一致)
            // 如需更精确计时, 可用 timeBeginPeriod(1) + Sleep
            QThread::msleep(10);
        }

        engine.shutdown();
    }

private:
    SharedState& shared_;
    std::atomic<bool> stop_requested_{false};
};
