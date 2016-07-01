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

#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

namespace signal_handler {
  /** \brief Signal handling for the ROS node wrapper
    * 
    * This class provides a static interface to bind the cleanup methods of
    * multiple ROS node implementations to a common process signal handler.
    */
  
  class SignalHandler {
  public:
    SignalHandler() = delete;

    typedef boost::function<void(int)> Handler;

    template <typename T> static void bind(int signal, void(T::*fp)(int),
      T* object);
    
    template <typename T> static void bindAll(void(T::*fp)(int), T* object);

    static void bind(int signal, const Handler& handler);
    
    template <typename T> static void unbind(int signal, void(T::*fp)(int),
      T* object);
    
    static void unbind(int signal, const Handler& handler);
    
  private:
    static std::map<int, std::list<Handler> > handlers;
    static boost::mutex mutex;
    
    static void signaled(int signal);
  };
};

#include <signal_handler/SignalHandler.tpp>
