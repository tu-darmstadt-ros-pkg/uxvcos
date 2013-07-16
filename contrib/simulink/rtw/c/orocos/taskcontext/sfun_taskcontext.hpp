#ifndef ORO_RTW_SFUN_TASKCONTEXT_H
#define ORO_RTW_SFUN_TASKCONTEXT_H

#include <rtt/TaskContext.hpp>
#include <rtt/Property.hpp>
#include <rtt/Port.hpp>
#include <rtt/Logger.hpp>

namespace RTW
{
    /**
     * This is set to \a this during component construction.
     * All external RTW functions called within the constructor can access
     * this variable to find the Component which is calling them. When the
     * constructor is left, currentTC is set to null again.
     * @see RTWComponent.cpp
     */
    extern RTT::TaskContext* currentTC;
}

using namespace RTT;

#endif
