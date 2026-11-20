from setuptools import setup

package_name = 'ftl_template'

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
        'Pacote modelo do ftl: um limitador de taxa com lógica pura, nó fino e '
        'parâmetro documentado, que cada fase copia para criar o seu.'),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'limiter = ftl_template.node:main',
        ],
    },
)
