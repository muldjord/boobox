/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/***************************************************************************
 *            boobox.h
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

#ifndef __BOOBOX_H__
#define __BOOBOX_H__

#include "settings.h"

#include <QObject>
#include <QCommandLineParser>
#include <QTcpServer>

class BooBox : public QObject
{
  Q_OBJECT

public:
  BooBox(const QCommandLineParser &parser);
  ~BooBox();

public slots:
  void run();

signals:
  void finished();

private slots:
  void handleConnection();
  void readData();
  void dataWritten(qint64 bytes);
  
private:
  Settings settings;
  QTcpServer *server = nullptr;
  QTcpSocket *connection = nullptr;
  QByteArray incomingData = "";
  qint64 outgoingSize = 0;
  qint64 bytesWritten = 0;
};

#endif // __BOOBOX_H__
