
#ifndef _INCLUDE_EXTENSION_PROPER_H_
#define _INCLUDE_EXTENSION_PROPER_H_

//This is a placeholder file for now

#include "smsdk_ext.h"
#include "surf_ramp_optimizer.h"

class SurfRampOptimizerExtension : public SDKExtension
{
public:
	virtual bool SDK_OnLoad(char* error, size_t maxlength, bool late);
	virtual void SDK_OnUnload();
	virtual void SDK_OnAllLoaded();
	virtual bool QueryRunning(char* error, size_t maxlength);
	virtual bool SDK_OnMetamodLoad(ISmmAPI* ismm, char* error, size_t maxlength, bool late);
};

#endif // _INCLUDE_EXTENSION_PROPER_H_