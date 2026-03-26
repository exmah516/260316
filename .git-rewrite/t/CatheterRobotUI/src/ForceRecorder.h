#pragma once
// ============================================================
// ForceRecorder.h
// 力信号 CSV 录制器
// ============================================================

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QStandardPaths>
#include "SharedState.h"

class ForceRecorder : public QObject
{
    Q_OBJECT

public:
    explicit ForceRecorder(QObject* parent = nullptr) : QObject(parent) {}

    bool isRecording() const { return file_ != nullptr; }

    QString currentFilePath() const { return filepath_; }

public slots:
    bool startRecording(const QString& dir = QString())
    {
        if (file_) return false;

        QString folder = dir.isEmpty()
            ? QStandardPaths::writableLocation(QStandardPaths::DesktopLocation)
            : dir;

        filepath_ = folder + "/force_"
            + QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss")
            + ".csv";

        file_ = new QFile(filepath_, this);
        if (!file_->open(QIODevice::WriteOnly | QIODevice::Text))
        {
            delete file_;
            file_ = nullptr;
            return false;
        }

        stream_ = new QTextStream(file_);
        *stream_ << "time_sec,fn_raw,ft_raw,fn_filtered,ft_filtered\n";
        sample_count_ = 0;
        return true;
    }

    void writeSamples(const std::deque<ForceSample>& samples)
    {
        if (!file_ || !stream_) return;
        for (const auto& s : samples)
        {
            *stream_ << QString::number(s.timestamp_sec, 'f', 4) << ","
                      << s.fn_raw << ","
                      << s.ft_raw << ","
                      << QString::number(s.fn_filtered, 'f', 4) << ","
                      << QString::number(s.ft_filtered, 'f', 4) << "\n";
            ++sample_count_;
        }
        // 每 500 条刷一次磁盘
        if (sample_count_ % 500 < samples.size())
            stream_->flush();
    }

    QString stopRecording()
    {
        if (!file_) return {};
        stream_->flush();
        delete stream_;
        stream_ = nullptr;
        file_->close();
        delete file_;
        file_ = nullptr;
        return filepath_;
    }

private:
    QFile*       file_    = nullptr;
    QTextStream* stream_  = nullptr;
    QString      filepath_;
    int          sample_count_ = 0;
};
