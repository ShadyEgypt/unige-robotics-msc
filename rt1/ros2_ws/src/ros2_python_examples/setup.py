from setuptools import find_packages, setup

package_name = 'ros2_python_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shady',
    maintainer_email='shadyrafat60@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_node = ros2_python_examples.publisher:main',
            'subscriber_node = ros2_python_examples.subscriber:main',
            'server_node = ros2_python_examples.server:main',
            'client_node = ros2_python_examples.client:main',
        ],
    },
)
