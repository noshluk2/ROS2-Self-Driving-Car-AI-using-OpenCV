from setuptools import find_packages, setup
import os
from glob import glob
import shutil
from setuptools.command.install import install
from ament_index_python.packages import get_package_share_directory


package_name = 'self_driving_car_pkg'


class CustomInstallCommand(install):
    def run(self):
        src_dir_abs = os.path.abspath('models')
        dst_dir_abs = os.path.abspath(os.path.join(get_package_share_directory(package_name),'models'))
        if os.path.exists(dst_dir_abs):
            shutil.rmtree(dst_dir_abs)
        shutil.copytree(src_dir_abs, dst_dir_abs)
        install.run(self)


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    cmdclass={
        'install': CustomInstallCommand,
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
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
            'drive_node = self_driving_car_pkg.drive_node:main',
            'vision_node = self_driving_car_pkg.vision_node:main',
            'spawner_node =  self_driving_car_pkg.sdf_spawner:main',

        ],
    },
)
