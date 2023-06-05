#include "mj_sim.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <cmath>
#include <iostream>
#include <thread>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

bool render_state = true;

void scanPluginLibraries()
{
  int nplugin = mjp_pluginCount();
  if(nplugin)
  {
    mc_rtc::log::info("[mc_mujoco] Built-in plugins");
    for(int i = 0; i < nplugin; ++i)
    {
      mc_rtc::log::info("  - {}", mjp_getPluginAtSlot(i)->name);
    }
  }

  const char * mujoco_plugin_path_char = std::getenv("MUJOCO_PLUGIN_PATH");
  if(!mujoco_plugin_path_char)
  {
    return;
  }
  std::stringstream mujoco_plugin_path_ss(mujoco_plugin_path_char);
  std::string plugin_path;
#ifdef _WIN32
  const char path_delimiter = ';';
#else
  const char path_delimiter = ':';
#endif
  while(std::getline(mujoco_plugin_path_ss, plugin_path, path_delimiter))
  {
    if(plugin_path.empty())
    {
      continue;
    }
    mc_rtc::log::info("[mc_mujoco] Scan plugins in {}", plugin_path);
    mj_loadAllPluginLibraries(
        plugin_path.c_str(), +[](const char * filename, int first, int count) {
          if(count == 0)
          {
            return;
          }
          mc_rtc::log::info("[mc_mujoco] Plugins registered by library {}", filename);
          for(int i = first; i < first + count; ++i)
          {
            mc_rtc::log::info("  - {}", mjp_getPluginAtSlot(i)->name);
          }
        });
  }
}

void simulate(mc_mujoco::MjSim & mj_sim)
{
  bool done = false;
  while(!done && render_state)
  {
    mj_sim.stepSimulation();
  }
}

int main(int argc, char * argv[])
{
  if(mc_rtc::MC_RTC_VERSION != mc_rtc::version())
  {
    mc_rtc::log::error("mc_mujoco was compiled with {} but mc_rtc is at version {}, you might "
                       "face subtle issues or unexpected crashes, please recompile mc_mujoco",
                       mc_rtc::MC_RTC_VERSION, mc_rtc::version());
  }

  mc_mujoco::MjConfiguration config;
  {
    po::options_description desc("mc_mujoco options");
    po::positional_options_description p;
    p.add("mc-config", 1);
    // clang-format off
    desc.add_options()
      ("help", "Show this help message")
      ("mc-config", po::value<std::string>(&config.mc_config), "Configuration given to mc_rtc")
      ("step-by-step", po::bool_switch(&config.step_by_step), "Start the simulation in step-by-step mode")
      ("torque-control", po::bool_switch(&config.torque_control), "Enable torque control")
      ("without-controller", po::bool_switch(), "Disable mc_rtc controller inside mc_mujoco")
      ("without-visualization", po::bool_switch(), "Disable mc_mujoco GUI")
      ("without-mc-rtc-gui", po::bool_switch(), "Disable mc_rtc GUI")
      ("with-collisions", po::bool_switch(), "Visualize collisions model")
      ("without-visuals", po::bool_switch(), "Disable visuals display")
      ("sync", po::bool_switch(&config.sync_real_time), "Synchronize mc_mujoco simulation time with real time");
    // clang-format on
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).positional(p).run(), vm);
    po::notify(vm);
    if(vm.count("help"))
    {
      std::cout << desc << "\n";
      return 0;
    }
    config.with_controller = !vm["without-controller"].as<bool>();
    config.with_visualization = !vm["without-visualization"].as<bool>();
    config.with_mc_rtc_gui = !vm["without-mc-rtc-gui"].as<bool>();
    if(!vm["without-visuals"].defaulted())
    {
      config.visualize_visual = !vm["without-visuals"].as<bool>();
    }
    if(!vm["with-collisions"].defaulted())
    {
      config.visualize_collisions = vm["with-collisions"].as<bool>();
    }
  }

  scanPluginLibraries();

  mc_mujoco::MjSim mj_sim(config);

  std::thread simThread(simulate, std::ref(mj_sim));

  while(render_state)
  {
    mj_sim.updateScene();
    render_state = mj_sim.render();
  }

  simThread.join();
  mj_sim.stopSimulation();
  return 0;
}
