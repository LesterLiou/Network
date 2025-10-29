from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'network_tester'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含 launch 檔案
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'psutil',
        'matplotlib'
    ],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Modularized Network Testing Tool for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 新的模組化版本
            'network_test_node = network_tester.node_network_tester:main',
            
            # 保留舊版本（向後相容）
            'network_test = network_tester.network_tester_node:main',
            'network_image = network_tester.network_tester_image:main',
            'network_rosbridge = network_tester.network_tester_rosbridge:main',
            'network_unified = network_tester.network_tester_unified:main',
            
            # 工具腳本
            'run_network_test = network_tester.scripts.run_network_test:main',
            'analyze_results = network_tester.scripts.analyze_results:main',
        ],
    },
)
