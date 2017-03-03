from conans import ConanFile, CMake
import os


class AlgorithmSDKConan(ConanFile):
    name = "AlgorithmSDK"
    version = "0.0.1"
    settings = {"os": ["Windows"],
                "compiler": {"Visual Studio": {"version": ['12', "14"]}},
                "arch": ['x86', 'x86_64'],
                "build_type": ['Release', 'Debug']}
    requires = (
        "CommonUtils/0.0.1@oslh/stable",
        "gtest/1.8.0@lasote/stable",
        "HVRLogger/0.1.5.0@oslh/stable",
        "OpenCV/3.1.0@oslh/stable",
        "VRTrackDevHAL/0.0.1@oslh/stable",
        "Clapack/0.0.1@oslh/stable",
    )
    generators = "cmake"
    exports = "CMakeLists.txt"
    options = {"build_develop": [True, False], "algorithmdll_prerelease": [True, False]}
    default_options = "build_develop=False", "algorithmdll_prerelease=True"

    def config_options(self):
        pass

    def build(self):
        version = "vs2013" if self.settings.compiler['Visual Studio'].version == 12 else "vs2015"
        target_directory = os.sep.join(["_build", version, str(self.settings.arch), str(self.settings.build_type), ])
        if self.settings.os == "Windows":
            self.run("IF not exist " + target_directory + " mkdir " + target_directory)
        cmake = CMake(self.settings)

        build_develop = "-DBUILD_DEVELOP=" + ("ON" if self.options.build_develop else "OFF")
        prerelease = "-DALGORITHMDLL_PRERELEASE=" + ("ON " if self.options.algorithmdll_prerelease else "OFF")
        cmake_flags = "%s %s" % (build_develop, prerelease)
        cd_build = "cd " + target_directory
        self.run(
            '%s && cmake %s %s %s' % (cd_build, cmake.command_line, os.sep.join(["..", "..", "..", "..", ]), cmake_flags))
        self.run("%s && cmake --build . %s  -- /maxcpucount" % (cd_build, cmake.build_config))

    def imports(self):
        version = "vs2013" if self.settings.compiler['Visual Studio'].version == 12 else "vs2015"
        target_directory = os.sep.join(["_build", version, str(self.settings.arch), str(self.settings.build_type), ])
        self.copy(pattern="*.dll", dst=os.sep.join([target_directory, "bin", ]), src="bin")  # From bin to bin
        self.copy(pattern="*.*", dst=os.sep.join([target_directory, "bin/platforms", ]),
                  src="plugins/platforms")  # From plugins/platforms
        self.copy(pattern="*.dylib*", dst=os.sep.join([target_directory, "bin", ]), src="lib")  # From lib to bin

    def package(self):
        self.copy(pattern="*.hpp", dst="include", src="AlgorithmSDK/include", keep_path=True)
        self.copy(pattern="*.hpp", dst="include", src=".", keep_path=False)
        self.copy(pattern="*.h", dst="include", src="AlgorithmSDK/include", keep_path=True)
        self.copy(pattern="*.h", dst="include", src=".", keep_path=False)
        self.copy(pattern="*.lib", dst="lib", src=".", keep_path=False)
        self.copy(pattern="*.dll", dst="bin", src=".", keep_path=False)
        self.copy(pattern="*.so*", dst="lib", src=".", keep_path=False)
        self.copy(pattern="*.dylib*", dst="lib", src=".", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["algorithmSDK"]
