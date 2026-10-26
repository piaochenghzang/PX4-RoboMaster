"""Existing local PX4 daemon wire contract; mocks only, no control connection."""
import os
import struct
import unittest
from pathlib import Path
from unittest.mock import patch
from native_client import NativeClient


class NativeTransportTests(unittest.TestCase):
    def client(self):return NativeClient(Path('/unused'),{},transport='unix',expected_pid=os.getpid())

    def connection(self,mock,parts):
        conn=mock.return_value.__enter__.return_value
        conn.getsockopt.return_value=struct.pack('3i',os.getpid(),os.geteuid(),0)
        conn.recv.side_effect=parts
        return conn

    def test_constructor_is_passive_and_requires_owned_pid(self):
        with patch('native_client.socket.socket') as mock:
            self.client();mock.assert_not_called()
            with self.assertRaises(ValueError):NativeClient(Path('/unused'),{},transport='unix')

    def test_fragmented_footer_and_same_status_parser(self):
        with patch('native_client.socket.socket') as mock:
            conn=self.connection(mock,[b'  timestamp: 1000\n  joint_target: [0.1, 0.2]\n',
                b'  feedback_valid: True\n',b'\x00',b'\x00',b''])
            status=self.client().status('arm_control_status')
            self.assertEqual(status,dict(timestamp=1000.,joint_target=[.1,.2],feedback_valid=True))
            conn.connect.assert_called_once_with('/tmp/px4-sock-0')
            conn.sendall.assert_called_once_with(b'listener arm_control_status 1\x00')

    def test_native_rejection_is_not_hidden(self):
        with patch('native_client.socket.socket') as mock:
            self.connection(mock,[b'guard rejected\n\x00\xff',b''])
            with self.assertRaisesRegex(RuntimeError,'guard rejected'):
                self.client().client('arm_control','fixed_grasp_start',1,2,3,4)

    def test_quaternion_annotation_is_not_parsed_as_array_data(self):
        with patch('native_client.socket.socket') as mock:
            self.connection(mock,[b'  timestamp: 1000\n  q: [1.0, 0.0, 0.0, 0.0] (Roll: 0.0 deg)\n\x00\x00',b''])
            self.assertEqual(self.client().status('vehicle_visual_odometry')['q'],[1.,0.,0.,0.])

    def test_wrong_process_or_uid_rejected_before_send(self):
        for peer in [(os.getpid()+1,os.geteuid(),0),(os.getpid(),os.geteuid()+1,0)]:
            with patch('native_client.socket.socket') as mock:
                conn=self.connection(mock,[]);conn.getsockopt.return_value=struct.pack('3i',*peer)
                with self.assertRaisesRegex(RuntimeError,'owned PX4'):
                    self.client().client('listener','arm_control_status',1)
                conn.sendall.assert_not_called()

    def test_ambiguous_or_unrelated_commands_rejected(self):
        with patch('native_client.socket.socket') as mock:
            for module,args in [('commander',['arm']),('listener',['bad topic',1]),
                ('arm_control',['fixed_grasp_start\n']),('listener',['\x00'])]:
                with self.assertRaises(ValueError):self.client().client(module,*args)
            mock.assert_not_called()

    def test_incomplete_or_oversize_reply_rejected(self):
        for parts in ([b'timestamp: 1\n',b''],[b'x'*1048577]):
            with patch('native_client.socket.socket') as mock:
                self.connection(mock,parts)
                with self.assertRaises(RuntimeError):self.client().client('listener','arm_control_status',1)


if __name__=='__main__':unittest.main()
