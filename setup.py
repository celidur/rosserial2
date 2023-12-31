from setuptools import setup

package_name = 'rosserial2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + ".rosserial_std_msgs", package_name + ".rosserial_sensor_msgs",
              package_name + ".rosserial_msgs", package_name + ".rosserial_geometry_msgs"],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gaetan',
    maintainer_email='github@gagolino.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosserial2 = rosserial2:main'
        ],
    },
)
