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
  loop.exec();
  emit finished();
}

void BooBox::handleConnection()
{
  printf("Incoming connection...\n");
  incomingData.clear();
  connection = server->nextPendingConnection();
  connect(connection, &QTcpSocket::readyRead, this, &BooBox::readData);
  connect(connection, &QTcpSocket::bytesWritten, this, &BooBox::dataWritten);
}

void BooBox::readData()
{
  incomingData = connection->readAll();
  printf("Data:\n'%s'\n", incomingData.data());
  if(incomingData.right(1) == "\n") {
    printf("Command:\n%s\n", incomingData.data());
  }
  bytesWritten = 0;
  QFile wavFile(settings.dataPath + "/hello.wav");
  if(wavFile.open(QIODevice::ReadOnly)) {
    QByteArray outgoingData = wavFile.readAll();
    outgoingData.remove(0, outgoingData.indexOf("data") + 8); // Also remove data size of 4 bytes
    wavFile.close();
    outgoingSize = outgoingData.size();
    connection->write(outgoingData);
  }
}


void BooBox::dataWritten(qint64 bytes)
{
  bytesWritten += bytes;
  if(bytesWritten == outgoingSize) {
    connection->close();
    connection->deleteLater();
  }
}
