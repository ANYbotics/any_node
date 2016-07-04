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

#include <csignal>
#include <ros/ros.h>

#include "signal_handler/SignalHandler.hpp"

namespace signal_handler {

/*****************************************************************************/
/* Static Member Initialization                                              */
/*****************************************************************************/

std::map<int, std::list<SignalHandler::Handler> > SignalHandler::handlers;
boost::mutex SignalHandler::mutex;

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void SignalHandler::bind(int signal_, const Handler& handler) {
  boost::mutex::scoped_lock lock(mutex);

  std::map<int, std::list<Handler> >::iterator it = handlers.find(signal_);
  if (it == handlers.end()) {
    it = handlers.insert(std::make_pair(signal_, std::list<Handler>())).first;
    signal(signal_, &SignalHandler::signaled);
  }
  
  for (std::list<Handler>::const_iterator jt = it->second.begin();
      jt != it->second.end(); ++jt)
    if (*jt == &handler)
      return;
  
  it->second.push_back(handler);
}

void SignalHandler::unbind(int signal_, const Handler& handler) {
  boost::mutex::scoped_lock lock(mutex);
  std::map<int, std::list<Handler> >::iterator it = handlers.find(signal_);
  
  if (it == handlers.end())
    return;
  
  for (std::list<Handler>::iterator jt = it->second.begin();
      jt != it->second.end(); ++jt) {
    if (*jt == &handler) {
      it->second.erase(jt);
      
      if (it->second.empty())
        signal(signal_, SIG_DFL);

      return;
    }
  }
}

void SignalHandler::signaled(int signal) {
  boost::mutex::scoped_lock lock(mutex);
  std::map<int, std::list<Handler> >::iterator it = handlers.find(signal);
  
  if (it == handlers.end())
    return;
  
  for (std::list<Handler>::iterator jt = it->second.begin();
      jt != it->second.end(); ++jt)
    (*jt)(signal);
}

}
