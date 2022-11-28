/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/***************************************************************************
 *            boobox.cpp
 *
 *  Wed Jul 27 09:49:00 CEST 2022
 *  Copyright 2022 Lars Muldjord
 *  muldjordlars@gmail.com
 ****************************************************************************/
/*
 *  This file is part of boobox.
 *
 *  boobox is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  boobox is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with boobox; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA.
 */

#include "boobox.h"

#include <QTcpSocket>
#include <QRandomGenerator>
#include <QFileInfo>
#include <QDir>
#include <QFile>

BooBox::BooBox(const QCommandLineParser &parser)
{
  if(parser.isSet("d") && !parser.value("d").isEmpty()) {
    settings.dataPath = parser.value("d");
  }
  if(parser.isSet("p") && !parser.value("p").isEmpty()) {
    settings.listenPort = parser.value("p").toInt();
  }
}

BooBox::~BooBox()
{
}

void BooBox::run()
{
  server = new QTcpServer();
  connect(server, &QTcpServer::newConnection, this, &BooBox::handleConnection);
  server->listen(QHostAddress(QHostAddress::Any), settings.listenPort);
  QEventLoop loop;
  printf("BooBox server v" VERSION " running!\n");
  loop.exec();
  emit finished();
}

void BooBox::handleConnection()
{
  printf("Incoming connection...\n");
  incomingData.clear();
  connection = server->nextPendingConnection();
  //connect(connection, &QTcpSocket::readyRead, this, &BooBox::readData);
  connect(connection, &QTcpSocket::bytesWritten, this, &BooBox::dataWritten);

  bytesWritten = 0;
  QDir dataDir(settings.dataPath, "*.wav", QDir::Name, QDir::Files);
  QList<QFileInfo> fileInfos = dataDir.entryInfoList();
  QString fileName = QFileInfo(fileInfos.at(QRandomGenerator::global()->bounded(fileInfos.count()))).absoluteFilePath();
  QFile wavFile(fileName);
  if(wavFile.open(QIODevice::ReadOnly)) {
    QByteArray outgoingData = wavFile.readAll();
    outgoingData.remove(0, outgoingData.indexOf("data") + 4); // + 4 is to also remove the char bytes for 'data'
    qint32 data_size = 0;
    memcpy(&data_size, outgoingData, 4);
    outgoingData.remove(0, 4); // Remove 4 byte that indicate data size
    outgoingData = outgoingData.left(data_size);
    wavFile.close();
    outgoingSize = outgoingData.size();
    printf("Sending: '%s'\n", qPrintable(fileName));
    connection->write(outgoingData);
  }
}

/*
void BooBox::readData()
{
  incomingData = connection->readAll();
  if(incomingData.right(1) == "\n") {
    printf("Command: '%s'\n", incomingData.trimmed().data());
  } else {
    return;
  }
  bytesWritten = 0;
  QDir dataDir(settings.dataPath, "*.wav", QDir::Name, QDir::Files);
  QList<QFileInfo> fileInfos = dataDir.entryInfoList();
  QString fileName = QFileInfo(fileInfos.at(QRandomGenerator::global()->bounded(fileInfos.count()))).absoluteFilePath();
  QFile wavFile(fileName);
  if(wavFile.open(QIODevice::ReadOnly)) {
    QByteArray outgoingData = wavFile.readAll();
    outgoingData.remove(0, outgoingData.indexOf("data") + 8); // Also remove data size of 4 bytes
    wavFile.close();
    outgoingSize = outgoingData.size();
    printf("Sending: '%s'\n", qPrintable(fileName));
    connection->write(outgoingData);
  }
}
*/

void BooBox::dataWritten(qint64 bytes)
{
  printf("Writing %llu bytes of data\n", bytes);
  bytesWritten += bytes;
  if(bytesWritten == outgoingSize) {
    printf("Done writing, closing connection...\n\n");
    connection->close();
    connection->deleteLater();
  }
}
