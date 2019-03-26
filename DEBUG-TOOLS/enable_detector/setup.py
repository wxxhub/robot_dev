from setuptools import setup

package_name = 'enable_detector'

setup(
    name=package_name,
    version='0.6.1',
    packages=[],
    py_modules=[
        'enable_detector',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='wxx',
    author_email='614610440@qq.com',
    maintainer='wxx',
    maintainer_email='614610440@qq.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of minimal publishers using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'enable_detector = enable_detector:main',
        ],
    },
)