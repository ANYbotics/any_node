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

#include <boost/bind.hpp>

namespace signal_handler {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T> void SignalHandler::bind(int signal, void(T::*fp)(int),
    T* object) {
  SignalHandler::bind(signal, boost::bind(fp, object, _1));
}
    
template <typename T> void SignalHandler::unbind(int signal, void(T::*fp)(int),
    T* object) {
  SignalHandler::unbind(signal, boost::bind(fp, object, _1));
}

template <typename T> void SignalHandler::bindAll(void(T::*fp)(int), T* object) {
  const Handler handler = boost::bind(fp, object, _1);
  SignalHandler::bind(SIGINT, handler);
  SignalHandler::bind(SIGTERM, handler); // shell command kill
  SignalHandler::bind(SIGABRT, handler); // invoked by abort();
  SignalHandler::bind(SIGFPE, handler);
  SignalHandler::bind(SIGILL, handler);
  SignalHandler::bind(SIGQUIT, handler); // the QUIT character, usually C-'\'
  SignalHandler::bind(SIGHUP, handler);  // hang-up” signal is used to report that the user’s terminal is disconnected
  // SIGKILL cannot be handled
}

}