import json
import tempfile
from mayfly import sensor

class SensorCapture:
    ALL_STREAM_IDS=list(range(64))

    def __init__(self, config_file, stream_ids=ALL_STREAM_IDS):
        self.app = sensor.AppSensor()
        self.queue = self.app.create_event_queue(stream_ids, 8)
        self.app.start(config_file)
    def __del__(self):
        self.app.stop()

    @classmethod
    def from_dict(cls, config):
        _,cfg_filename=tempfile.mkstemp('_mayfly.json')
        with open(cfg_filename, 'w') as fh:
            json.dump(config, fh, indent=2)
        return cls(cfg_filename, cls.ALL_STREAM_IDS)

    @staticmethod
    def _find_fields(list_or_dict, field):
        found = []
        if isinstance(list_or_dict, dict):
            for k, v in list_or_dict.items():
                if k == field:
                    found.append((list_or_dict, v))
                elif isinstance(v, dict) or isinstance(v, list):
                    found += SensorCapture._find_fields(v, field)
        elif isinstance(list_or_dict, list):
            for v in list_or_dict:
                found += SensorCapture._find_fields(v, field)
        return found

    @staticmethod
    def _frame_data_fixup(frame_and_data):
        if 'source' not in frame_and_data:
            print('skipping %s' % frame_and_data)
            return None
        source = frame_and_data['source']
        tensors = frame_and_data.get('$data',[])
        found = SensorCapture._find_fields(source, '$data')
        assert len(found) == len(tensors), "Error found %d references to %d tensors." % (len(found), len(tensors))

        for (parent,obj) in found:
            parent[obj['name']] = tensors[obj['idx']]
            del parent['$data']
        return source

    def read(self, blocking=True, wait_ms=1000):
        if blocking:
            while True:
                res = self.queue.wait(1000*wait_ms)
                if res is None:
                    continue
                source = SensorCapture._frame_data_fixup(res)
                if source is None:
                    continue
                return source;
        else:
            while True:
                res = self.queue.wait(1000*wait_ms)
                if res is None:
                    return None
                source = SensorCapture._frame_data_fixup(res)
                if source is None:
                    return None
                return source;
