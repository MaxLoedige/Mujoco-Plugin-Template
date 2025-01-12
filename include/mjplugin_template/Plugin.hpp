#ifndef MUJOCO_PLUGIN_TEMPLATE
#define MUJOCO_PLUGIN_TEMPLATE

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjplugin.h>
#include <memory>

template <typename Plugin>
struct MJPlugin {

    virtual MJPlugin(const mjModel* m, mjData* d, int instance){
        this->create(m,d,instance);
    }

    static std::unique_ptr<Plugin> Create(const mjModel* m, mjData* d, int instance) {
        return std::make_unique<Plugin>(m, d, instance);
    }


    static void RegisterPlugin(){
        mjp_defaultPlugin(&info);

        this->registerAttributes();

        info.init = +[](const mjModel* m, mjData* d, int instance) {
            std::unique_ptr<Plugin> plugin = Plugin::Create(m,d, instance);
            if (plugin == nullptr) {return -1;}
            d->plugin_data[instance] = reinterpret_cast<uintptr_t>(ws.release());
            plugin->init();
            return 0;
        };
        info.destroy = +[](mjData* d, int instance) {
            delete reinterpret_cast<Plugin*>(d->plugin_data[instance]);
            plugin->destroy();
            d->plugin_data[instance] = 0;
        };
        // Reset callback.
        info.reset = +[](const mjModel* m, mjtNum* plugin_state, void* plugin_data,
                            int instance) {
            auto* plugin = reinterpret_cast<Plugin*>(plugin_data);
            plugin->reset(m, instance);
        };

        // Compute callback.
        info.compute = +[](const mjModel* m, mjData* d, int instance, int capability_bit) {
                auto* plugin = reinterpret_cast<Plugin*>(d->plugin_data[instance]);
                plugin->compute(m, d, instance);
            };

        info.advance = +[](const mjModel* m, mjData* d, int instance) {
            auto* plugin = reinterpret_cast<Plugin*>(d->plugin_data[instance]);
            plugin->advance(m, d, instance);
        };
        // Register the plugin.
        mjp_registerPlugin(&plugin);

    }



    mjpPlugin info;

    virtual void registerAttributes(){
        info.name = "mujoco.plugin.template";
        info.capabilityflags |= mjPLUGIN_SENSOR;
        std::vector<const char*> attributes = {};
        info.nattribute = attributes.size();
        info.attributes = attributes.data();
    }

    virtual void nstate(const mjModel* m, int instance){return 0;}
    virtual void nsensordata (const mjModel* m, int instance, int sensor_id) {};
    
    virtual void create(const mjModel* m, mjData* d, int instance) {}

    virtual void init(const mjModel* m, mjData* d, int instance) {}

    virtual void destroy(mjData* d, int instance) {}
    virtual void reset(const mjModel* m, mjData* d, void* plugin_data,
                        int instance) {}
    virtual void compute(const mjModel* m, mjData* d, int instance, int capability_bit){}
    virtual void advance(const mjModel* m, mjData* d, int instance){}



};

#endif