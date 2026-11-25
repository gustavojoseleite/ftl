from setuptools import setup

package_name = 'ftl_trajectory'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo',
    maintainer_email='gujoseleite@gmail.com',
    description=(
        'Gerenciador de trajetória do ftl: amostragem espacial adaptativa, CCMA e o trilho '
        'único do comboio, publicado pelo primeiro trator e seguido por todos.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trail_publisher = ftl_trajectory.trail_publisher:main',
            'trail_follower = ftl_trajectory.trail_follower:main',
        ],
    },
)
