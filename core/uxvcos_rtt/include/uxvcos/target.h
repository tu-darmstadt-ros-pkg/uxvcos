#ifndef UXVCOS_TARGET_H
#define UXVCOS_TARGET_H

#if !defined(TARGET_NAME) && defined(ROS_PACKAGE_NAME)
  #define TARGET_NAME     ROS_PACKAGE_NAME
#elif defined(TARGET_NAME)

#else
  #pragma message("Warning: No TARGET_NAME defined!")
#endif

#endif // UXVCOS_TARGET_H
