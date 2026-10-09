"""当前 PLC 离线测试共用的最小 ST 解析器。"""
import re


class ST:
    def __init__(self, text):
        text = re.sub(r"//[^\n]*", "", text)
        self.tokens = re.findall(
            r"T#\d+(?:MS|S)|16#[0-9A-F]+|\d+\.\d+|\d+|[A-Za-z_]\w*|:=|<>|<=|>=|[^\s]",
            text,
        )
        self.i = 0

    def take(self):
        token = self.tokens[self.i]
        self.i += 1
        return token

    def peek(self):
        return self.tokens[self.i] if self.i < len(self.tokens) else ""

    def until(self, end):
        values = []
        depth = 0
        while self.peek():
            if self.peek() == end and depth == 0:
                self.take()
                return values
            token = self.take()
            if token in ("(", "["):
                depth += 1
            elif token in (")", "]"):
                depth -= 1
            values.append(token)
        raise AssertionError(f"Missing {end}")

    @staticmethod
    def expr(tokens):
        mapping = {
            "AND": "and",
            "OR": "or",
            "NOT": "not",
            "TRUE": "True",
            "FALSE": "False",
            "<>": "!=",
            "=": "==",
            ":=": "=",
        }

        def convert(token):
            if token.startswith("16#"):
                return str(int(token[3:], 16))
            if token.startswith("T#"):
                return str(int(re.search(r"\d+", token)[0]) * (1 if token.endswith("MS") else 1000))
            return mapping.get(token, token)

        return " ".join(convert(token) for token in tokens)

    def block(self, end=(), depth=0):
        output = []
        indent = "    " * depth
        while self.peek() and self.peek() not in end:
            token = self.take()
            if token == "IF":
                output.append(indent + "if " + self.expr(self.until("THEN")) + ":")
                output += self.block(("ELSIF", "ELSE", "END_IF"), depth + 1)
                while self.peek() == "ELSIF":
                    self.take()
                    output.append(indent + "elif " + self.expr(self.until("THEN")) + ":")
                    output += self.block(("ELSIF", "ELSE", "END_IF"), depth + 1)
                if self.peek() == "ELSE":
                    self.take()
                    output.append(indent + "else:")
                    output += self.block(("END_IF",), depth + 1)
                assert self.take() == "END_IF"
            elif token == "FOR":
                name = self.take()
                assert self.take() == ":="
                start = self.expr(self.until("TO"))
                finish = self.expr(self.until("DO"))
                output.append(indent + f"for {name} in range(int({start}), int({finish}) + 1):")
                output += self.block(("END_FOR",), depth + 1)
                assert self.take() == "END_FOR"
            elif token == "CASE":
                value = self.expr(self.until("OF"))
                cases = []
                level = 0
                begin = self.i
                while not (self.peek() == "END_CASE" and level == 0):
                    label = self.take()
                    if label == "CASE":
                        level += 1
                    if label == "END_CASE":
                        level -= 1
                    if level == 0 and label.isdigit() and self.peek() == ":":
                        cases.append((int(label), self.i - 1))
                finish = self.i
                self.take()
                for index, (label, position) in enumerate(cases):
                    boundary = cases[index + 1][1] if index + 1 < len(cases) else finish
                    branch = ST("")
                    branch.tokens = self.tokens[position + 2:boundary]
                    output.append(indent + ("if " if index == 0 else "elif ") + f"({value}) == {label}:")
                    output += branch.block(depth=depth + 1)
                assert cases, f"Empty CASE at {begin}"
            elif token == ";":
                continue
            else:
                statement = [token] + self.until(";")
                nesting = 0
                assignment = None
                for index, value in enumerate(statement):
                    if value in ("(", "["):
                        nesting += 1
                    elif value in (")", "]"):
                        nesting -= 1
                    elif value == ":=" and nesting == 0:
                        assignment = index
                        break
                if assignment is None:
                    output.append(indent + self.expr(statement))
                else:
                    output.append(
                        indent
                        + self.expr(statement[:assignment])
                        + " = "
                        + self.expr(statement[assignment + 1:])
                    )
        return output or [indent + "pass"]


class Timer:
    def __init__(self):
        self.Q = False
        self.elapsed = 0

    def __call__(self, IN, PT):
        self.elapsed = self.elapsed + 1 if IN else 0
        self.Q = IN and self.elapsed >= PT
