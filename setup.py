from setuptools import setup
import glob

package_name = 'advanced_nodes'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.py')),  # Install launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dvat162518',
    maintainer_email='danialvishwa543@gmail.com',
    description='AMR ROS2 Advanced Nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_publisher = advanced_nodes.topic_publisher:main',
            'topic_subscriber = advanced_nodes.topic_subscriber:main',
            'service_server = advanced_nodes.service_server:main',
            'service_client = advanced_nodes.service_client:main',
            'action_server = advanced_nodes.action_server:main',
            'action_client = advanced_nodes.action_client:main',
            'lifecycle_node = advanced_nodes.lifecycle_node:main',
        ],
    },
)
