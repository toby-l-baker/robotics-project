# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Fraunhofer FKIE/US, Alexander Tiderko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Fraunhofer nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division, absolute_import, print_function, unicode_literals

from python_qt_binding.QtGui import QColor
import os
import roslib
import rospy

from fkie_master_discovery.common import masteruri_from_ros
from fkie_node_manager_daemon.common import utf8
from fkie_node_manager_daemon import screen
from fkie_node_manager_daemon import settings as nmd_settings
from fkie_node_manager.common import get_ros_home


class LoggingConfig(object):
    '''
    Holds the settings for changed log level while start a node.
    '''
    LOGLEVEL = 'DEFAULT'
    LOGLEVEL_ROSCPP = 'INFO'
    LOGLEVEL_SUPERDEBUG = 'WARN'
    CONSOLE_FORMAT = 'DEFAULT'

    def __init__(self):
        self.loglevel = self.LOGLEVEL
        self.loglevel_roscpp = self.LOGLEVEL_ROSCPP
        self.loglevel_superdebug = self.LOGLEVEL_SUPERDEBUG
        self.console_format = self.CONSOLE_FORMAT

    def get_attributes(self):
        '''
        Returns a list with all possible attributes, e.g.: loglevel, console_format

        :rtype: [str]
        '''
        return ['loglevel',
                'loglevel_roscpp',
                'loglevel_superdebug',
                'console_format'
                ]

    def is_default(self, attribute):
        '''
        Checks for an attribute :meth:`get_attributes` however it was changed.
        '''
        return getattr(self, attribute) == getattr(self, attribute.upper())

    def get_alternatives(self, attribute):
        '''
        Retruns a list with all possible values for an attribute :meth:`get_attributes`.

        :rtype: [str]
        '''
        result = []
        if attribute == 'console_format':
            result = [self.CONSOLE_FORMAT,
                      '[${severity}] [${time}]: ${message}',
                      '[${severity}] [${time}] [${logger}]: ${message}']
        elif attribute == 'loglevel':
            result = ['DEFAULT', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL']
        elif attribute == 'loglevel_roscpp':
            result = ['INFO', 'DEBUG', 'WARN', 'ERROR', 'FATAL']
        elif attribute == 'loglevel_superdebug':
            result = ['WARN', 'DEBUG', 'INFO', 'ERROR', 'FATAL']
        if not self.is_default(attribute):
            result.insert(0, getattr(self, attribute))
        return result


class Settings(object):

    USER_DEFAULT = 'robot'
    ''':ivar USER_DEFAULT: default user, if no one is set for specific master. Defaul: robot'''
    # set the cwd to the package of the fkie_node_manager to support the images
    # in HTML descriptions of the robots and capabilities
    PKG_NAME = 'fkie_node_manager'
    try:
        PACKAGE_DIR = roslib.packages.get_pkg_dir(PKG_NAME)
    except Exception:
        PACKAGE_DIR = "%s/../.." % os.path.realpath(os.path.dirname(__file__))
        if "dist-packages" in __file__:
            PACKAGE_DIR = "%s/../../share/fkie_node_manager" % PACKAGE_DIR
        print("PACKAGE_DIR: %s" % PACKAGE_DIR)
    ROBOTS_DIR = os.path.join(PACKAGE_DIR, 'images')
    CFG_PATH = os.path.expanduser('~/.config/ros.fkie/node_manager/')
    ''':ivar CFG_PATH: configuration path to store the settings and history'''
    HELP_FILE = os.path.join(PACKAGE_DIR, 'README.rst')
    CURRENT_DIALOG_PATH = os.path.expanduser('~')
    LOG_PATH = screen.LOG_PATH

    LOG_VIEWER = "/usr/bin/less -fKLnQrSU"
    STARTER_SCRIPT = 'rosrun fkie_node_manager remote_nm.py'
    ''':ivar STARTER_SCRIPT: the script used on remote hosts to start new ROS nodes.'''

    LAUNCH_HISTORY_FILE = 'launch.history'
    LAUNCH_HISTORY_LENGTH = 5

    PARAM_HISTORY_FILE = 'param.history'
    PARAM_HISTORY_LENGTH = 12

    CFG_REDIRECT_FILE = 'redirect'
    CFG_FILE = 'settings.ini'
    CFG_GUI_FILE = 'settings.ini'

    TIMEOUT_CONTROL = 5
    TIMEOUT_UPDATES = 20

    SEARCH_IN_EXT = ['.launch', '.yaml', '.conf', '.cfg', '.iface', '.nmprofile', '.sync', '.test', '.xml', '.xacro']
    LAUNCH_VIEW_EXT = ['.launch', '.yaml', '.conf', '.cfg', '.iface', '.nmprofile', '.sync', '.test', '.xacro']

    STORE_GEOMETRY = True
    MOVABLE_DOCK_WIDGETS = True
    AUTOUPDATE = True
    MAX_TIMEDIFF = 0.5

    START_SYNC_WITH_DISCOVERY = False
    CONFIRM_EXIT_WHEN_CLOSING = True
    HIGHLIGHT_XML_BLOCKS = True
    COLORIZE_HOSTS = True
    CHECK_FOR_NODELETS_AT_START = True
    SHOW_NOSCREEN_ERROR = True
    ASK_RELOAD_LAUNCH = True

    SHOW_DOMAIN_SUFFIX = False

    TRANSPOSE_PUB_SUB_DESCR = True
    TIMEOUT_CLOSE_DIALOG = 30.0
    GROUP_BY_NAMESPACE = True
    TIMEOUT_GRPC = nmd_settings.GRPC_TIMEOUT
    SYSMON_DEFAULT_INTERVAL = 10

    DEAFULT_HOST_COLORS = [QColor(255, 255, 235).rgb()]

    def __init__(self):
        self.reload()

    def reload(self):
        '''
        Loads the settings from file or sets default values if no one exists.
        '''
        self._terminal_emulator = None
        self._terminal_command_arg = 'e'
        self._noclose_str = '-hold'
        self._terminal_title = '--title'
        self._masteruri = masteruri_from_ros()
        # self.CFG_PATH = os.path.join(get_ros_home(), 'node_manager')
        self.CFG_PATH = os.path.expanduser('~/.config/ros.fkie/node_manager/')
        # loads the current configuration path. If the path was changed, a redirection
        # file exists with new configuration folder
        if not os.path.isdir(self.CFG_PATH):
            os.makedirs(self.CFG_PATH)
            self._cfg_path = self.CFG_PATH
        else:
            settings = self.qsettings(os.path.join(self.CFG_PATH, self.CFG_REDIRECT_FILE))
            self._cfg_path = settings.value('cfg_path', self.CFG_PATH)
        # move all stuff from old location to new
        try:
            import shutil
            old_cfg_path = os.path.join(get_ros_home(), 'node_manager')
            if os.path.exists(old_cfg_path):
                print("move configuration to new destination: %s" % self.CFG_PATH)
                for filename in os.listdir(old_cfg_path):
                    shutil.move(os.path.join(old_cfg_path, filename), os.path.join(self.CFG_PATH, filename))
                shutil.rmtree(old_cfg_path)
        except Exception:
            pass
        print("Configuration path: %s" % self.CFG_PATH)
        # after the settings path was loaded, load other settings
        self._robots_path = self.ROBOTS_DIR
        settings = self.qsettings(self.CFG_FILE)
        self._default_user = settings.value('default_user', self.USER_DEFAULT)
        settings.beginGroup('default_user_hosts')
        self._default_user_hosts = dict()
        for k in settings.childKeys():
            self._default_user_hosts[k] = settings.value(k, self._default_user)
        settings.endGroup()
        try:
            self._launch_history_length = int(settings.value('launch_history_length', self.LAUNCH_HISTORY_LENGTH))
        except Exception:
            self._launch_history_length = self.LAUNCH_HISTORY_LENGTH
        try:
            self._param_history_length = int(settings.value('param_history_length', self.PARAM_HISTORY_LENGTH))
        except Exception:
            self._param_history_length = self.PARAM_HISTORY_LENGTH
        self._current_dialog_path = self.CURRENT_DIALOG_PATH
        self._log_viewer = self.LOG_VIEWER
        self._start_remote_script = self.STARTER_SCRIPT
        self._launch_view_file_ext = self.str2list(settings.value('launch_view_file_ext', ', '.join(self.LAUNCH_VIEW_EXT)))
        self._store_geometry = self.str2bool(settings.value('store_geometry', self.STORE_GEOMETRY))
        self._movable_dock_widgets = self.str2bool(settings.value('movable_dock_widgets', self.MOVABLE_DOCK_WIDGETS))
        self.SEARCH_IN_EXT = list(set(self.SEARCH_IN_EXT) | set(self._launch_view_file_ext))
        self._autoupdate = self.str2bool(settings.value('autoupdate', self.AUTOUPDATE))
        self._max_timediff = float(settings.value('max_timediff', self.MAX_TIMEDIFF))
        self._rosconsole_cfg_file = 'rosconsole.config'
        self.logging = LoggingConfig()
        self.logging.loglevel = settings.value('logging/level', LoggingConfig.LOGLEVEL)
        self.logging.loglevel_roscpp = settings.value('logging/level_roscpp', LoggingConfig.LOGLEVEL_ROSCPP)
        self.logging.loglevel_superdebug = settings.value('logging/level_superdebug', LoggingConfig.LOGLEVEL_SUPERDEBUG)
        self.logging.console_format = settings.value('logging/rosconsole_format', LoggingConfig.CONSOLE_FORMAT)
        self._start_sync_with_discovery = self.str2bool(settings.value('start_sync_with_discovery', self.START_SYNC_WITH_DISCOVERY))
        self._confirm_exit_when_closing = self.str2bool(settings.value('confirm_exit_when_closing', self.CONFIRM_EXIT_WHEN_CLOSING))
        self._highlight_xml_blocks = self.str2bool(settings.value('highlight_xml_blocks', self.HIGHLIGHT_XML_BLOCKS))
        self._colorize_hosts = self.str2bool(settings.value('colorize_hosts', self.COLORIZE_HOSTS))
        self._check_for_nodelets_at_start = self.str2bool(settings.value('check_for_nodelets_at_start', self.CHECK_FOR_NODELETS_AT_START))
        self._show_noscreen_error = self.str2bool(settings.value('show_noscreen_error', self.SHOW_NOSCREEN_ERROR))
        self._ask_reload_launch = self.str2bool(settings.value('ask_reload_launch', self.ASK_RELOAD_LAUNCH))
        self._show_domain_suffix = self.str2bool(settings.value('show_domain_suffix', self.SHOW_DOMAIN_SUFFIX))
        self._transpose_pub_sub_descr = self.str2bool(settings.value('transpose_pub_sub_descr', self.TRANSPOSE_PUB_SUB_DESCR))
        self._timeout_close_dialog = float(settings.value('timeout_close_dialog', self.TIMEOUT_CLOSE_DIALOG))
        self._group_nodes_by_namespace = self.str2bool(settings.value('group_nodes_by_namespace', self.GROUP_BY_NAMESPACE))
        self._timeout_grpc = float(settings.value('timeout_grpc', self.TIMEOUT_GRPC))
        self._sysmon_default_interval = int(settings.value('sysmon_default_interval', self.SYSMON_DEFAULT_INTERVAL))
        nmd_settings.GRPC_TIMEOUT = self._timeout_grpc
        settings.beginGroup('host_colors')
        self._host_colors = dict()
        for k in settings.childKeys():
            self._host_colors[k] = settings.value(k, None)
        settings.endGroup()
        self.init_hosts_color_list()
        self._launch_history = None  # list with file names

    def masteruri(self):
        return self._masteruri

    @property
    def cfg_path(self):
        return self._cfg_path

    @cfg_path.setter
    def cfg_path(self, path):
        if path:
            abspath = os.path.abspath(path).rstrip(os.path.sep)
            if not os.path.isdir(abspath):
                os.makedirs(abspath)
            self._cfg_path = abspath
            if abspath != os.path.abspath(self.CFG_PATH).rstrip(os.path.sep):
                settings = self.qsettings(os.path.join(self.CFG_PATH, self.CFG_REDIRECT_FILE))
                settings.setValue('cfg_path', abspath)
            else:
                # remove the redirection
                settings = self.qsettings(os.path.join(self.CFG_PATH, self.CFG_REDIRECT_FILE))
                settings.remove('cfg_path')
            self.reload()

    @property
    def robots_path(self):
        return self._robots_path

    @robots_path.setter
    def robots_path(self, path):
        if path:
            if not os.path.isdir(path):
                os.makedirs(path)
            self._robots_path = path
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('robots_path', self._robots_path)

    @property
    def default_user(self):
        return self._default_user

    @default_user.setter
    def default_user(self, user):
        if user:
            self._default_user = user
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('default_user', self._default_user)

    def host_user(self, host):
        if host in self._default_user_hosts:
            return self._default_user_hosts[host]
        return self.default_user

    def set_host_user(self, host, user):
        if host and user:
            self._default_user_hosts[host] = user
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('default_user_hosts/%s' % host, user)

    @property
    def launch_history_length(self):
        return self._launch_history_length

    @launch_history_length.setter
    def launch_history_length(self, length):
        self._launch_history_length = length
        settings = self.qsettings(self.CFG_FILE)
        settings.setValue('launch_history_length', self._launch_history_length)

    @property
    def param_history_length(self):
        return self._param_history_length

    @param_history_length.setter
    def param_history_length(self, length):
        self._param_history_length = length
        settings = self.qsettings(self.CFG_FILE)
        settings.setValue('param_history_length', self._param_history_length)

    @property
    def current_dialog_path(self):
        return self._current_dialog_path

    @current_dialog_path.setter
    def current_dialog_path(self, path):
        self._current_dialog_path = path

    def robot_image_file(self, robot_name):
        return os.path.join(self.ROBOTS_DIR, '%s.png' % robot_name)

    @property
    def log_viewer(self):
        return self._log_viewer

    @log_viewer.setter
    def log_viewer(self, viewer):
        self._log_viewer = viewer

    @property
    def start_remote_script(self):
        return self._start_remote_script

    @start_remote_script.setter
    def start_remote_script(self, script):
        self._start_remote_script = script

    @property
    def launch_view_file_ext(self):
        return self._launch_view_file_ext

    @launch_view_file_ext.setter
    def launch_view_file_ext(self, exts):
        self._launch_view_file_ext = self.str2list('%s' % exts)
        settings = self.qsettings(self.CFG_FILE)
        settings.setValue('launch_view_file_ext', self._launch_view_file_ext)
        self.SEARCH_IN_EXT = list(set(self.SEARCH_IN_EXT) | set(self._launch_view_file_ext))

    @property
    def store_geometry(self):
        return self._store_geometry

    @store_geometry.setter
    def store_geometry(self, value):
        v = self.str2bool(value)
        if self._store_geometry != v:
            self._store_geometry = v
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('store_geometry', self._store_geometry)

    @property
    def movable_dock_widgets(self):
        return self._movable_dock_widgets

    @movable_dock_widgets.setter
    def movable_dock_widgets(self, value):
        v = self.str2bool(value)
        if self._movable_dock_widgets != v:
            self._movable_dock_widgets = v
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('movable_dock_widgets', self._movable_dock_widgets)

    @property
    def autoupdate(self):
        return self._autoupdate

    @autoupdate.setter
    def autoupdate(self, value):
        v = self.str2bool(value)
        if self._autoupdate != v:
            self._autoupdate = v
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('autoupdate', self._autoupdate)

    @property
    def max_timediff(self):
        return self._max_timediff

    @max_timediff.setter
    def max_timediff(self, value):
        v = float(value)
        if self._max_timediff != v:
            self._max_timediff = v
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('max_timediff', self._max_timediff)

    def rosconsole_cfg_file(self, package):
        result = os.path.join(self.LOG_PATH, '%s.%s' % (package, self._rosconsole_cfg_file))
        with open(result, 'w') as cfg_file:
            cfg_file.write('log4j.logger.ros=%s\n' % self.logging.loglevel)
            cfg_file.write('log4j.logger.ros.roscpp=%s\n' % self.logging.loglevel_roscpp)
            cfg_file.write('log4j.logger.ros.roscpp.superdebug=%s\n' % self.logging.loglevel_superdebug)
        return result

    def store_logging(self):
        settings = self.qsettings(self.CFG_FILE)
        settings.setValue('logging/level', self.logging.loglevel)
        settings.setValue('logging/level_roscpp', self.logging.loglevel_roscpp)
        settings.setValue('logging/level_superdebug', self.logging.loglevel_superdebug)
        settings.setValue('logging/rosconsole_format', self.logging.console_format)

    @property
    def start_sync_with_discovery(self):
        return self._start_sync_with_discovery

    @start_sync_with_discovery.setter
    def start_sync_with_discovery(self, value):
        v = self.str2bool(value)
        if self._start_sync_with_discovery != v:
            self._start_sync_with_discovery = v
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('start_sync_with_discovery', self._start_sync_with_discovery)

    @property
    def confirm_exit_when_closing(self):
        return self._confirm_exit_when_closing

    @confirm_exit_when_closing.setter
    def confirm_exit_when_closing(self, value):
        val = self.str2bool(value)
        if self._confirm_exit_when_closing != val:
            self._confirm_exit_when_closing = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('confirm_exit_when_closing', self._confirm_exit_when_closing)

    @property
    def highlight_xml_blocks(self):
        return self._highlight_xml_blocks

    @highlight_xml_blocks.setter
    def highlight_xml_blocks(self, value):
        val = self.str2bool(value)
        if self._highlight_xml_blocks != val:
            self._highlight_xml_blocks = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('highlight_xml_blocks', self._highlight_xml_blocks)

    @property
    def colorize_hosts(self):
        return self._colorize_hosts

    @colorize_hosts.setter
    def colorize_hosts(self, value):
        val = self.str2bool(value)
        if self._colorize_hosts != val:
            self._colorize_hosts = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('colorize_hosts', self._colorize_hosts)

    @property
    def check_for_nodelets_at_start(self):
        return self._check_for_nodelets_at_start

    @check_for_nodelets_at_start.setter
    def check_for_nodelets_at_start(self, value):
        val = self.str2bool(value)
        if self._check_for_nodelets_at_start != val:
            self._check_for_nodelets_at_start = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('check_for_nodelets_at_start', self._check_for_nodelets_at_start)

    @property
    def show_noscreen_error(self):
        return self._show_noscreen_error

    @show_noscreen_error.setter
    def show_noscreen_error(self, value):
        val = self.str2bool(value)
        if self._show_noscreen_error != val:
            self._show_noscreen_error = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('show_noscreen_error', self._show_noscreen_error)

    @property
    def ask_reload_launch(self):
        return self._ask_reload_launch

    @ask_reload_launch.setter
    def ask_reload_launch(self, value):
        val = self.str2bool(value)
        if self._ask_reload_launch != val:
            self._ask_reload_launch = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('ask_reload_launch', self._ask_reload_launch)

    @property
    def show_domain_suffix(self):
        return self._show_domain_suffix

    @show_domain_suffix.setter
    def show_domain_suffix(self, value):
        val = self.str2bool(value)
        if self._show_domain_suffix != val:
            self._show_domain_suffix = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('show_domain_suffix', self._show_domain_suffix)

    @property
    def transpose_pub_sub_descr(self):
        return self._transpose_pub_sub_descr

    @transpose_pub_sub_descr.setter
    def transpose_pub_sub_descr(self, value):
        val = self.str2bool(value)
        if self._transpose_pub_sub_descr != val:
            self._transpose_pub_sub_descr = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('transpose_pub_sub_descr', self._transpose_pub_sub_descr)

    @property
    def timeout_close_dialog(self):
        return self._timeout_close_dialog

    @timeout_close_dialog.setter
    def timeout_close_dialog(self, value):
        v = float(value)
        if self._timeout_close_dialog != v:
            self._timeout_close_dialog = v
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('timeout_close_dialog', self._timeout_close_dialog)

    @property
    def timeout_grpc(self):
        return self._timeout_grpc

    @timeout_grpc.setter
    def timeout_grpc(self, value):
        v = float(value)
        if self._timeout_grpc != v:
            self._timeout_grpc = v
            nmd_settings.GRPC_TIMEOUT = self._timeout_grpc
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('timeout_grpc', self._timeout_grpc)

    @property
    def sysmon_default_interval(self):
        return self._sysmon_default_interval

    @sysmon_default_interval.setter
    def sysmon_default_interval(self, length):
        self._sysmon_default_interval = length
        settings = self.qsettings(self.CFG_FILE)
        settings.setValue('sysmon_default_interval', self._sysmon_default_interval)

    @property
    def group_nodes_by_namespace(self):
        return self._group_nodes_by_namespace

    @group_nodes_by_namespace.setter
    def group_nodes_by_namespace(self, value):
        val = self.str2bool(value)
        if self._group_nodes_by_namespace != val:
            self._group_nodes_by_namespace = val
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('group_nodes_by_namespace', self._group_nodes_by_namespace)

    def host_color(self, host, default_color):
        if self.colorize_hosts:
            # get the color from settings file
            if host in self._host_colors:
                result = self._host_colors[host]
                if isinstance(result, (unicode, str)):
                    return long(result)
                return result
            else:
                # determine a color
                index = abs(hash(host)) % len(self.DEAFULT_HOST_COLORS)
                return self.DEAFULT_HOST_COLORS[index]
        return default_color

    def set_host_color(self, host, color):
        if host and color:
            self._host_colors[host] = color
            settings = self.qsettings(self.CFG_FILE)
            settings.setValue('host_colors/%s' % host, color)

    @property
    def launch_history(self):
        '''
        Read the history of the recently loaded files from the file stored in ROS_HOME path.

        :return: the list with file names
        :rtype: list(str)
        '''
        if self._launch_history is not None:
            return self._launch_history
        # load history from file, only first time
        result = list()
        history_file = self.qsettings(self.LAUNCH_HISTORY_FILE)
        size = history_file.beginReadArray("launch_history")
        for i in range(size):
            history_file.setArrayIndex(i)
            if i >= self.launch_history_length:
                break
            launch_file = history_file.value("file")
            # TODO: if os.path.isfile(launch_file):
            result.append(launch_file)
        history_file.endArray()
        self._launch_history = result
        return self._launch_history

    def launch_history_add(self, path, replace=None):
        '''
        Adds a path to the list of recently loaded files.

        :param str path: the path with the file name
        :param str replace: the path to replace, e.g. rename
        '''
        to_remove = replace
        if replace is None:
            to_remove = path
        if self._launch_history is None:
            self.launch_history
        try:
            self._launch_history.remove(to_remove)
        except Exception:
            pass
        self._launch_history.append(path)
        while len(self._launch_history) > self.launch_history_length:
            self._launch_history.pop(0)
        self._launch_history_save(self._launch_history)

    def launch_history_remove(self, path):
        '''
        Removes a path from the list of recently loaded files.

        :param list(str) path: the path with the file name
        '''
        try:
            self._launch_history.remove(path)
            self._launch_history_save(self._launch_history)
        except Exception:
            pass

    def _launch_history_save(self, paths):
        '''
        Saves the list of recently loaded files to history. The existing history will be replaced!

        :param list(str) paths: the list with filenames
        '''
        history_file = self.qsettings(self.LAUNCH_HISTORY_FILE)
        history_file.beginWriteArray("launch_history")
        for i, launch_file in enumerate(paths):
            history_file.setArrayIndex(i)
            history_file.setValue("file", launch_file)
        history_file.endArray()
        self._launch_history = list(paths)

    def str2bool(self, v):
        if isinstance(v, bool):
            return v
        return v.lower() in ("yes", "true", "t", "1")

    def str2list(self, lstr):
        if isinstance(lstr, list):
            return lstr
        try:
            lstr = lstr.strip('[]')
            lstr = lstr.replace('u"', '')
            lstr = lstr.replace('"', '')
            lstr = lstr.replace("'", '')
            lstr = lstr.replace(",", ' ')
            return [utf8(i).strip() for i in lstr.split(' ') if i]
        except Exception:
            return []

    def terminal_cmd(self, cmd, title, noclose=False):
        '''
        Creates a command string to run with a terminal prefix

        :param list(str) cmd: the list with a command and args
        :param str title: the title of the terminal
        :return: command with a terminal prefix
        :rtype: str
        '''
        if self._terminal_emulator is None:
            self._terminal_emulator = ""
            for t in ['/usr/bin/x-terminal-emulator', '/usr/bin/xterm', '/opt/x11/bin/xterm']:
                if os.path.isfile(t) and os.access(t, os.X_OK):
                    # workaround to support the command parameter in different terminal
                    if os.path.basename(os.path.realpath(t)) in ['terminator', 'gnome-terminal', 'xfce4-terminal']:
                        self._terminal_command_arg = 'x'
                    else:
                        self._terminal_command_arg = 'e'
                    if os.path.basename(os.path.realpath(t)) in ['terminator', 'gnome-terminal', 'gnome-terminal.wrapper']:
                        self._noclose_str = '--profile hold'
                        if noclose:
                            rospy.loginfo("If your terminal close after the execution, you can change this behavior in "
                                          "profiles. You can also create a profile with name 'hold'. This profile will "
                                          "be then load by node_manager.")
                    elif os.path.basename(os.path.realpath(t)) in ['xfce4-terminal']:
                        self._noclose_str = ''
                        self._terminal_title = '-T'
                    self._terminal_emulator = t
                    break
        if self._terminal_emulator == "":
            raise Exception("No Terminal found! Please install one of ['/usr/bin/x-terminal-emulator', '/usr/bin/xterm', '/opt/x11/bin/xterm']")
        self._noclose_str = self._noclose_str if noclose else ""
        return '%s %s "%s" %s -%s %s' % (self._terminal_emulator, self._terminal_title, title, self._noclose_str, self._terminal_command_arg, ' '.join(cmd))

    def qsettings(self, settings_file):
        from python_qt_binding.QtCore import QSettings
        path = settings_file
        if not settings_file.startswith(os.path.sep):
            path = os.path.join(self.cfg_path, settings_file)
        return QSettings(path, QSettings.IniFormat)

    def init_hosts_color_list(self):
        self.DEAFULT_HOST_COLORS = [
            QColor(255, 255, 235).rgb(),
            QColor(87, 93, 94).rgb(),
            QColor(205, 186, 136).rgb(),
            QColor(249, 168, 0).rgb(),
            QColor(232, 140, 0).rgb(),
            QColor(175, 128, 79).rgb(),
            QColor(221, 175, 39).rgb(),
            QColor(227, 217, 198).rgb(),
            QColor(186, 72, 27).rgb(),
            QColor(246, 120, 40).rgb(),
            QColor(255, 77, 6).rgb(),
            QColor(89, 25, 31).rgb(),
            QColor(216, 160, 166).rgb(),
            QColor(129, 97, 131).rgb(),
            QColor(196, 97, 140).rgb(),
            QColor(118, 104, 154).rgb(),
            QColor(188, 64, 119).rgb(),
            QColor(0, 56, 123).rgb(),
            QColor(15, 76, 100).rgb(),
            QColor(0, 137, 182).rgb(),
            QColor(99, 125, 150).rgb(),
            QColor(5, 139, 140).rgb(),
            QColor(34, 45, 90).rgb(),
            QColor(60, 116, 96).rgb(),
            QColor(54, 103, 53).rgb(),
            QColor(80, 83, 60).rgb(),
            QColor(17, 66, 50).rgb(),
            QColor(108, 124, 89).rgb(),
            QColor(97, 153, 59).rgb(),
            QColor(185, 206, 172).rgb(),
            QColor(0, 131, 81).rgb(),
            QColor(126, 186, 181).rgb(),
            QColor(0, 181, 26).rgb(),
            QColor(122, 136, 142).rgb(),
            QColor(108, 110, 107).rgb(),
            QColor(118, 106, 94).rgb(),
            QColor(56, 62, 66).rgb(),
            QColor(128, 128, 118).rgb(),
            QColor(127, 130, 116).rgb(),
            QColor(197, 199, 196).rgb(),
            QColor(137, 105, 62).rgb(),
            QColor(112, 69, 42).rgb(),
            QColor(141, 73, 49).rgb(),
            QColor(90, 56, 38).rgb(),
            QColor(233, 224, 210).rgb(),
            QColor(236, 236, 231).rgb(),
            QColor(43, 43, 44).rgb(),
            QColor(121, 123, 122).rgb()
        ]
