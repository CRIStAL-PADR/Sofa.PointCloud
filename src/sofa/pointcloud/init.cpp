#include <sofa/core/ObjectFactory.h>
#include <sofa/pointcloud/config.h>

extern "C" {
    SOFA_POINTCLOUD_API void initExternalModule();
    SOFA_POINTCLOUD_API const char* getModuleName();
    SOFA_POINTCLOUD_API const char* getModuleVersion();
    SOFA_POINTCLOUD_API const char* getModuleLicense();
    SOFA_POINTCLOUD_API const char* getModuleDescription();
    SOFA_POINTCLOUD_API void registerObjects(sofa::core::ObjectFactory* factory);
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }
}

const char* getModuleName()
{
    return sofa_tostring(SOFA_TARGET);
}

const char* getModuleLicense()
{
    return "";
}

const char* getModuleVersion()
{
    return sofa_tostring(SOFA_POINTCLOUD_VERSION);
}

const char* getModuleDescription()
{
    return "Toolbox to manipulate gaussian splated point clouds";
}

void registerObjects(sofa::core::ObjectFactory* factory)
{
}
