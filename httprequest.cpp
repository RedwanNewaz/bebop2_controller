#include "httprequest.h"

HttpRequest::HttpRequest(QObject *parent) : QObject(parent)
{
    manager_ = new QNetworkAccessManager();
       QObject::connect(manager_, &QNetworkAccessManager::finished,
           this, [=](QNetworkReply *reply) {
               if (reply->error()) {
                   qDebug() << reply->errorString();
                   emit response(reply->errorString());
                   return;
               }

               QString answer = reply->readAll();
               emit response(answer);
               qDebug() << answer;
           }
       );

}

void HttpRequest::send(const QString &url)
{
    request_.setUrl(QUrl(url));
    manager_->get(request_);
}
