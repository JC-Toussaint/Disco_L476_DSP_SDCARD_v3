
/**-----------------------------------------------------------------
    @file       time_f.h
    @brief      
    @details    
    @author     Falko Bilz
    @version    $Rev::  $
    @date       $Date:: $
    @copyright  (c) Copyright by Falko Bilz 2007 - 2014
    @copyright  All rights reserved
------------------------------------------------------------------*/

#pragma once

#include <stdbool.h>    // bool definition
#include <stdint.h>     // uint32_t definition

void    time_start_ms   ( volatile uint32_t* ref );
bool    time_end_ms     ( volatile uint32_t* ref, const uint32_t del );
