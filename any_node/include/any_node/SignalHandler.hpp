/******************************************************************************
* Copyright (C) 2014 by Ralf Kaestner                                        *
* ralf.kaestner@gmail.com                                                    *
*                                                                            *
* This program is free software; you can redistribute it and/or modify       *
* it under the terms of the Lesser GNU General Public License as published by*
* the Free Software Foundation; either version 3 of the License, or          *
* (at your option) any later version.                                        *
*                                                                            *
* This program is distributed in the hope that it will be useful,            *
* but WITHOUT ANY WARRANTY; without even the implied warranty of             *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
* Lesser GNU General Public License for more details.                        *
*                                                                            *
* You should have received a copy of the Lesser GNU General Public License   *
* along with this program. If not, see <http://www.gnu.org/licenses/>.       *
******************************************************************************/

/** \file Signal.h
* \brief Header file providing the Signal class interface
*/

#pragma once

#include <map>
#include <list>
#include <csignal>
#include <functional>
#include <mutex>

namespace any_node {
/** \brief Signal handling for the ROS node wrapper
*
* This class provides a static interface to bind the cleanup methods of
* multiple ROS node implementations to a common process signal handler.
*/

class SignalHandler {
public:
  SignalHandler() = delete;

  using Handler = std::function<void(int)>;

  template <typename T> static void bind(int signal, void(T::*fp)(int), T* object) {
      SignalHandler::bind(signal, std::bind(fp, object, std::placeholders::_1));
  }

  static void bind(int signal, const Handler& handler);

  template <typename T> static void bindAll(void(T::*fp)(int), T* object) {
      const Handler handler = std::bind(fp, object, std::placeholders::_1);
      SignalHandler::bind(SIGINT, handler);
      SignalHandler::bind(SIGTERM, handler); // shell command kill
      SignalHandler::bind(SIGABRT, handler); // invoked by abort();
      SignalHandler::bind(SIGFPE, handler);
      SignalHandler::bind(SIGILL, handler);
      SignalHandler::bind(SIGQUIT, handler); // the QUIT character, usually C-'\'
      SignalHandler::bind(SIGHUP, handler);  // hang-up” signal is used to report that the user’s terminal is disconnected
      // SIGKILL cannot be handled
  }

  template <typename T> static void unbind(int signal, void(T::*fp)(int), T* object) {
      SignalHandler::unbind(signal, std::bind(fp, object, std::placeholders::_1));
  }

  static void unbind(int signal, const Handler& handler);

  template <typename T> static void unbindAll(void(T::*fp)(int), T* object) {
      const Handler handler = std::bind(fp, object, std::placeholders::_1);
      SignalHandler::unbind(SIGINT, handler);
      SignalHandler::unbind(SIGTERM, handler);
      SignalHandler::unbind(SIGABRT, handler);
      SignalHandler::unbind(SIGFPE, handler);
      SignalHandler::unbind(SIGILL, handler);
      SignalHandler::unbind(SIGQUIT, handler);
      SignalHandler::unbind(SIGHUP, handler);
  }

private:
  static std::map<int, std::list<Handler> > handlers;
  static std::mutex mutex;

  static void signaled(int signal);
};

} // namespace any_node
