px4_add_module(
    MODULE drivers__ftservo
    MAIN ftservo
    SRCS
        ftservo.cpp
        
        ft_lib/SCS.cpp
        ft_lib/SCSerial.cpp
        ft_lib/SMS_STS.cpp

    INCLUDES
        ft_lib
        
    DEPENDS
        px4_work_queue
)