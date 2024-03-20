#ifndef HTTPREQUEST_H
#define HTTPREQUEST_H

#include <QtCore>
#include <QObject>
#include <QDebug>
#include <QUrl>
#include <QNetworkAccessManager>
#include <QNetworkReply>

class HttpRequest : public QObject
{
    Q_OBJECT
public:
    explicit HttpRequest(QObject *parent = nullptr);

    void send(const QString& url);

signals:
    void response(QString);

private:
    QNetworkAccessManager * manager_;
    QNetworkRequest request_;

};

#endif // HTTPREQUEST_H
