from setuptools import setup

package_name = 'ftl_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/localization.yaml']),
        ('share/' + package_name + '/launch', ['launch/localization.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo',
    maintainer_email='gujoseleite@gmail.com',
    description='Localização por veículo: o UKF do robot_localization publica o ftl/state.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_odometry = ftl_localization.gnss_odometry:main',
        ],
    },
)
