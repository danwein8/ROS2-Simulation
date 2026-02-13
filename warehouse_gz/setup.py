from setuptools import find_packages, setup

package_name = 'warehouse_gz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sim.launch.py']),
        ('share/' + package_name + '/launch', ['launch/spawn.launch.py']),
        ('share/' + package_name + '/launch', ['launch/gazebo_model.launch.py']),
        ('share/' + package_name + '/config', ['config/warehouse.yaml']),
        ('share/' + package_name + '/config', ['config/bridge_parameters.yaml']),
        ('share/' + package_name + '/models/simple_bot', ['models/simple_bot/robot.xacro']),
        ('share/' + package_name + '/models/simple_bot', ['models/simple_bot/robot.gazebo']),
        ('share/' + package_name + '/worlds', []),
        ('share/' + package_name + '/models/simple_bot', ['models/simple_bot/model.sdf']),
        ('share/' + package_name + '/warehouse_gz', ['warehouse_gz/gen_world.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dan',
    maintainer_email='Danweiner9@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
