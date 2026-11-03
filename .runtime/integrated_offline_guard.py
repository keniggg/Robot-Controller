"""Python diagnostics inside the runner's private bwrap namespace.

The kernel namespace/private /dev also isolate native code and subprocesses.
Selected HTTP tests can reach only a live listener bound by this process.
"""
import errno
import json
import os
import socket
import sys
import threading
import time
import weakref


class OfflineExternalBoundaryViolation(BaseException):
    pass


def forbidden(*args, **kwargs):
    with open(os.environ['OFFLINE_BOUNDARY_LOG'], 'a') as stream:
        stream.write(json.dumps(dict(at_ns=time.time_ns(), pid=os.getpid(),
                                     args=repr(args)[:600]))+'\n')
    raise OfflineExternalBoundaryViolation('unsimulated external boundary')


_namespace = os.readlink('/proc/self/ns/net')
_isolated = (os.environ.get('INTEGRATED_SANDBOX') == '1'
             and _namespace != os.environ.get('INTEGRATED_HOST_NET_NS'))
if not _isolated:
    forbidden('private network namespace missing')

_allow_listener = os.environ.get('OFFLINE_ALLOW_OWN_LOOPBACK') == '1'
_listeners = weakref.WeakKeyDictionary()
_listener_lock = threading.RLock()
_bind, _close = socket.socket.bind, socket.socket.close
_connect, _connect_ex = socket.socket.connect, socket.socket.connect_ex
_create_connection, _getaddrinfo = socket.create_connection, socket.getaddrinfo


def _registered(address):
    if not (_allow_listener and isinstance(address, tuple) and len(address) == 2
            and address[0] == '127.0.0.1' and type(address[1]) is int):
        return False
    with _listener_lock:
        return any(endpoint == address and listener.fileno() >= 0
                   for listener, endpoint in list(_listeners.items()))


def _bind_private(self, address):
    if not (_allow_listener and self.family == socket.AF_INET
            and address == ('127.0.0.1', 0)):
        forbidden('socket.bind', address)
    result = _bind(self, address)
    endpoint = self.getsockname()
    with _listener_lock:
        _listeners[self] = endpoint
    with open(os.environ['OFFLINE_FIXTURE_NETWORK_LOG'], 'a') as stream:
        stream.write(json.dumps(dict(pid=os.getpid(), namespace=_namespace,
                                     endpoint=endpoint, operation='bind'))+'\n')
    return result


def _close_private(self):
    with _listener_lock:
        _listeners.pop(self, None)
    return _close(self)


def _connect_private(self, address):
    if not _registered(address):
        forbidden('socket.connect', address)
    return _connect(self, address)


def _connect_ex_private(self, address):
    if not _registered(address):
        forbidden('socket.connect_ex', address)
    return _connect_ex(self, address)


def _create_private(address, *args, **kwargs):
    if _allow_listener and address == ('127.0.0.1', 0):
        # The service-cleanup fixture uses port zero as a guaranteed absent
        # endpoint. Preserve that refusal without making any network call.
        with open(os.environ['OFFLINE_FIXTURE_NETWORK_LOG'], 'a') as stream:
            stream.write(json.dumps(dict(pid=os.getpid(), namespace=_namespace,
                endpoint=address, operation='inert_port_zero_refusal', socket_syscall=False))+'\n')
        raise ConnectionRefusedError(errno.ECONNREFUSED, 'port zero has no fixture listener')
    if not _registered(address):
        forbidden('socket.create_connection', address)
    return _create_connection(address, *args, **kwargs)


def _resolve_private(host, port, *args, **kwargs):
    if not _registered((host, port)):
        forbidden('socket.getaddrinfo', host, port)
    return _getaddrinfo(host, port, *args, **kwargs)


def _reverse_private(host):
    # HTTPServer obtains a display name; no resolver traffic is needed.
    if _allow_listener and host == '127.0.0.1':
        return ('localhost', [], ['127.0.0.1'])
    forbidden('socket.gethostbyaddr', host)


socket.socket.bind = _bind_private
socket.socket.close = _close_private
socket.socket.connect = _connect_private
socket.socket.connect_ex = _connect_ex_private
socket.socket.sendto = forbidden
socket.create_connection = _create_private
socket.getaddrinfo = _resolve_private
socket.gethostbyaddr = _reverse_private
socket.gethostbyname = forbidden
socket.gethostbyname_ex = forbidden


def device_open(event, args):
    if event != 'open' or not args or not isinstance(args[0], (str, bytes)):
        return
    path = os.path.realpath(os.fsdecode(args[0]))
    if not path.startswith('/dev/'):
        return
    virtual_files = ('/dev/null', '/dev/zero', '/dev/random', '/dev/urandom', '/dev/ptmx')
    virtual_directories = ('/dev/shm/', '/dev/fd/', '/dev/pts/')
    if path not in virtual_files and not path.startswith(virtual_directories):
        forbidden('physical_device_open', path)
sys.addaudithook(device_open)
with open(os.environ['OFFLINE_SANDBOX_LOG'], 'a') as stream:
    stream.write(json.dumps(dict(pid=os.getpid(), net_namespace=_namespace,
        host_net_namespace=os.environ['INTEGRATED_HOST_NET_NS'],
        mount_namespace=os.readlink('/proc/self/ns/mnt'),
        pid_namespace=os.readlink('/proc/self/ns/pid'),
        private_namespace_verified=True, allow_own_loopback=_allow_listener))+'\n')

import handoff_import_audit
handoff_import_audit.install()
