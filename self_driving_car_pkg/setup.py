from setuptools import setup
import os
from glob import glob
# ======= Computer Vision Python Module's Path ===============
package_name = 'self_driving_car_pkg'
det_l_module ="self_driving_car_pkg/Detection/Lanes"
det_s_module ="self_driving_car_pkg/Detection/Signs"
config_module = "self_driving_car_pkg/config" 
data_module ="self_driving_car_pkg/data"
control_module ="self_driving_car_pkg/Control"
detec_l_a_module="self_driving_car_pkg/Detection/Lanes/a_Segmentation"
detec_l_b_module="self_driving_car_pkg/Detection/Lanes/b_Estimation"
detec_l_c_module="self_driving_car_pkg/Detection/Lanes/c_Cleaning"
detec_l_d_module="self_driving_car_pkg/Detection/Lanes/d_LaneInfo_Extraction"


detec_s_a_module="self_driving_car_pkg/Detection/Signs/a_Localization"
detec_s_b_module="self_driving_car_pkg/Detection/Signs/b_Classification"
detec_s_c_module="self_driving_car_pkg/Detection/Signs/c_Tracking"

package_name = 'self_driving_car_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,detec_s_a_module,detec_s_b_module,detec_s_c_module,detec_l_d_module,detec_l_c_module,detec_l_b_module,config_module,det_l_module,det_s_module,data_module,control_module,detec_l_a_module],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name,'launch'), glob('launch/*')),
    (os.path.join('share', package_name,'meshes/sign_board_traffic_light'), glob('meshes/sign_board_traffic_light/*')),
    (os.path.join('share', package_name,'meshes/tesla_mini'), glob('meshes/tesla_mini/*')),
    (os.path.join('share', package_name,'meshes/track'), glob('meshes/track/*')),
    (os.path.join('share', package_name,'worlds/'), glob('worlds/*')),


    (os.path.join('lib', package_name), glob('scripts/*')),


    # (os.path.join('share', package_name,'models'), glob('models/*')),
    (os.path.join('share', package_name,'worlds'), glob(' worlds/*')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='luqman',
    maintainer_email='noshluk2@gmail.com',
    description='This Package is for the explaination of course on Self Driving car UDEMY',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'self_driving_node = self_driving_car_pkg.computer_vision_node:main',
            'sdf_spawning_node = self_driving_car_pkg.sdf_spawner:main',

        ],
    },
)