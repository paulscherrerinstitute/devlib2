/*************************************************************************\
* Copyright (c) 2006 The University of Chicago, as Operator of Argonne
*     National Laboratory.
* EPICS BASE Versions 3.13.7
* and higher are distributed subject to a Software License Agreement found
* in file LICENSE that is included with this distribution.
\*************************************************************************/

/* $Id: devLibVMEOSD.c,v 1.2 2014/11/24 09:44:59 zimoch Exp $
 * This is a copy of the similarly named file from EPICS Base
 */

#include <stdlib.h>

#include "devLib.h"

/* This file must contain no definitions other than the following: */

devLibVirtualOS *pdevLibVME2=NULL;
