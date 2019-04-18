from setuptools import setup

package_name = 'robot_web'

setup(
    name = package_name,
    version = '0.0.0',
    packages = ['app'],
    py_modules = [
        'robot_web',
    ],
    install_requires = ['setuptools'],
    zip_safe = True,
    author = 'wxx',
    author_email = "wxx0520@foxmaile.com",
    keywords=['ROS', 'ROS2'],
    license = "WIT AI Lab",
    tests_require = ['pytest'],
    entry_points = {
        'console_scripts': [
            'robot_web = robot_web:main',
        ],
    }
)