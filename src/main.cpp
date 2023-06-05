#include "mj_sim.h"

#include <mc_rtc/logging.h>
#include <mc_rtc/version.h>

#include <cmath>
#include <iostream>
#include <thread>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

bool render_state = true;

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir()
{
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while(!success)
    {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if(!realpath)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if(written < buf_size)
      {
        success = true;
      }
      else if(written == buf_size)
      {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
      else
      {
        std::cerr << "failed to retrieve executable path: " << GetLastError() << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#  if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if(!buf)
    {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if(_NSGetExecutablePath(buf.get(), &buf_size))
    {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char * path = buf.get();
#  else
  const char * path = "/proc/self/exe";
#  endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while(!success)
    {
      realpath.reset(new(std::nothrow) char[buf_size]);
      if(!realpath)
      {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if(written < buf_size)
      {
        realpath.get()[written] = '\0';
        success = true;
      }
      else if(written == -1)
      {
        if(errno == EINVAL)
        {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: " << strerror(errno) << '\n';
        return "";
      }
      else
      {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if(realpath.empty())
  {
    return "";
  }

  for(std::size_t i = realpath.size() - 1; i > 0; --i)
  {
    if(realpath.c_str()[i] == kPathSep)
    {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries()
{
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if(nplugin)
  {
    std::printf("Built-in plugins:\n");
    for(int i = 0; i < nplugin; ++i)
    {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif

  // try to open the ${EXECDIR}/plugin directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  const std::string executable_dir = getExecutableDir();
  if(executable_dir.empty())
  {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char * filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for(int i = first; i < first + count; ++i)
        {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
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
