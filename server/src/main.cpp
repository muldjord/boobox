/* -*- Mode: C++; tab-width: 2; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/***************************************************************************
 *            main.cpp
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

#include <QtGlobal>

#include <QCoreApplication>
#include <QDir>
#include <QtDebug>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QTimer>

#include "boobox.h"

int main(int argc, char *argv[])
{
  QCoreApplication app(argc, argv);
  app.setApplicationVersion(VERSION);

  QDir::setCurrent(QDir::currentPath());

  QCommandLineParser parser;

  QString headerString = "BooBox v" VERSION;

  parser.setApplicationDescription("BooBox TCP server.");
  parser.addHelpOption();
  parser.addVersionOption();
  QCommandLineOption dOption("d", "Base data path (default is './data')", "PATH", "");
  parser.addOption(dOption);
  QCommandLineOption pOption("p", "Listen port (default is 4242)", "PORT", "");
  parser.addOption(pOption);

  parser.process(app);

  if(parser.isSet("help") || parser.isSet("h")) {
    parser.showHelp();
  } else {
    BooBox *x = new BooBox(parser);
    QObject::connect(x, &BooBox::finished, &app, &QCoreApplication::quit);
    QTimer::singleShot(0, x, SLOT(run()));
  }
  return app.exec();
}
