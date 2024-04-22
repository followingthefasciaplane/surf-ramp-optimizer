#include "extension.h"

//This is a placeholder file for now

SurfRampOptimizer g_SurfRampOptimizer;
SMEXT_LINK(&g_SurfRampOptimizer);

bool SurfRampOptimizerExtension::SDK_OnLoad(char* error, size_t maxlength, bool late)
{
	return g_SurfRampOptimizer.SDK_OnLoad(error, maxlength, late);
}

void SurfRampOptimizerExtension::SDK_OnUnload()
{
	g_SurfRampOptimizer.SDK_OnUnload();
}

void SurfRampOptimizerExtension::SDK_OnAllLoaded()
{
	g_SurfRampOptimizer.SDK_OnAllLoaded();
}

bool SurfRampOptimizerExtension::QueryRunning(char* error, size_t maxlength)
{
	return g_SurfRampOptimizer.QueryRunning(error, maxlength);
}

bool SurfRampOptimizerExtension::SDK_OnMetamodLoad(ISmmAPI* ismm, char* error, size_t maxlength, bool late)
{
	return g_SurfRampOptimizer.SDK_OnMetamodLoad(ismm, error, maxlength, late);
}