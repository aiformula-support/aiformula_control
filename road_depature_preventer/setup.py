from setuptools import find_packages, setup
from common_python.setup_util import get_data_files

package_name = 'road_depature_preventer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    # ],
    data_files=get_data_files(package_name, ("config", "launch",)),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebita',
    maintainer_email='sora_ebita@jp.honda',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'road_depature_preventer = road_depature_preventer.road_depature_preventer:main'
        ],
    },
)
