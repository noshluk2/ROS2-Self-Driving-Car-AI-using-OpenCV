from setuptools import setup
import os
from glob import glob 


package_name = 'self_driving_car_pkg'
config_module = "self_driving_car_pkg/config" 
det_module= "self_driving_car_pkg/Detection"
det_l_module= "self_driving_car_pkg/Detection/Lanes"


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,config_module,det_module,det_l_module],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
            ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'spawner_node = self_driving_car_pkg.sdf_spawner:main',
        'computer_vision_node = self_driving_car_pkg.computer_vision_node:main',
        'video_recording_node = self_driving_car_pkg.video_save:main',
        ],
    },
)
