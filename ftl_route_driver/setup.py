from setuptools import setup

package_name = 'ftl_route_driver'

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
        'Motorista roteirizado do líder: percorre uma rota conhecida do mundo com pure pursuit '
        'e perfil de velocidade, e publica o erro lateral e o progresso.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_driver = ftl_route_driver.node:main',
        ],
    },
)
