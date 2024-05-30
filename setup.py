from setuptools import find_packages, setup

package_name = 'autonomous_tb'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/data_collection_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sohan',
    maintainer_email='sohan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collection_node = autonomous_tb.data_collection_node:main',
        ],
    },
)
