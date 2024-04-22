#ifndef _INCLUDE_SOURCEMOD_EXTENSION_PROPER_H_
#define _INCLUDE_SOURCEMOD_EXTENSION_PROPER_H_

#include "smsdk_config.h"

#include <ISmmPlugin.h>
#include <IShareSys.h>
#include <IExtensionSys.h>
#include <IForwardSys.h>
#include <IPluginSys.h>
#include <IPlayerHelpers.h>

#include <iplayerinfo.h>
#include <ISDKTools.h>

#include <vector>
#include <unordered_map>
#include <mutex>

#define SMEXT_PLUGIN_NAME       "SurfRampOptimizer"
#define SMEXT_PLUGIN_AUTHOR     "jessetooler"
#define SMEXT_PLUGIN_DESCRIPTION  "pathfinder for surf ramps"
#define SMEXT_PLUGIN_VERSION    "1.0.0.0"
#define SMEXT_PLUGIN_URL        "https://gcpdot.com"

class SurfRampOptimizer : public ISmmPlugin
{
public:
	bool Load(PluginId id, ISmmAPI* ismm, char* error, size_t maxlen, bool late);
	bool Unload(char* error, size_t maxlen);
	bool Pause(char* error, size_t maxlen);
	bool Unpause(char* error, size_t maxlen);
	void AllPluginsLoaded();

	const char* GetAuthor();
	const char* GetName();
	const char* GetDescription();
	const char* GetURL();
	const char* GetLicense();
	const char* GetVersion();
	const char* GetDate();
};

#endif // _INCLUDE_SOURCEMOD_EXTENSION_PROPER_H_#pragma once
